use std::collections::HashMap;
use walrus::ir::{
    BinaryOp, Call, Instr, InstrSeq, LoadKind, MemArg, VisitorMut, dfs_pre_order_mut,
};
use walrus::{ConstExpr, FunctionBuilder, FunctionId, GlobalId, MemoryId, Module, ValType};

const PAGE_EXPORT: &str = "__waveshark_page_thread";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [input, output] = args.as_slice() else {
        return Err("usage: webspin <in.wasm> <out.wasm>".into());
    };
    let mut module = Module::from_file(input)?;
    let replaced = rewrite(&mut module)?;
    module.emit_wasm_file(output)?;
    eprintln!("webspin: {replaced} waits spin on the page's thread");
    Ok(())
}

fn rewrite(module: &mut Module) -> Result<usize, Box<dyn std::error::Error>> {
    if module.exports.iter().any(|e| e.name == PAGE_EXPORT) {
        return Ok(0);
    }
    let page = module.globals.add_local(
        ValType::I32,
        true,
        false,
        ConstExpr::Value(walrus::ir::Value::I32(0)),
    );
    let mut mark = FunctionBuilder::new(&mut module.types, &[], &[]);
    mark.func_body().i32_const(1).global_set(page);
    let mark = mark.finish(vec![], &mut module.funcs);
    module.exports.add(PAGE_EXPORT, mark);

    let mut helpers: HashMap<(MemoryId, u64, bool), FunctionId> = HashMap::new();
    let mut found: Vec<(MemoryId, MemArg, bool)> = Vec::new();
    for (_, f) in module.funcs.iter_local() {
        let mut v = Find(&mut found);
        walrus::ir::dfs_in_order(&mut v, f, f.entry_block());
    }
    for (memory, arg, wide) in found {
        helpers
            .entry((memory, arg.offset, wide))
            .or_insert_with(|| helper(module, page, memory, arg, wide));
    }
    let skip: Vec<FunctionId> = helpers.values().copied().collect();
    let mut replaced = 0;
    let ids: Vec<FunctionId> = module.funcs.iter_local().map(|(id, _)| id).collect();
    for id in ids {
        if skip.contains(&id) {
            continue;
        }
        let f = module.funcs.get_mut(id).kind.unwrap_local_mut();
        let entry = f.entry_block();
        let mut v = Swap { helpers: &helpers, replaced: 0 };
        dfs_pre_order_mut(&mut v, f, entry);
        replaced += v.replaced;
    }
    Ok(replaced)
}

struct Find<'a>(&'a mut Vec<(MemoryId, MemArg, bool)>);

impl<'instr> walrus::ir::Visitor<'instr> for Find<'_> {
    fn visit_atomic_wait(&mut self, w: &walrus::ir::AtomicWait) {
        self.0.push((w.memory, w.arg, w.sixty_four));
    }
}

struct Swap<'a> {
    helpers: &'a HashMap<(MemoryId, u64, bool), FunctionId>,
    replaced: usize,
}

impl VisitorMut for Swap<'_> {
    fn start_instr_seq_mut(&mut self, seq: &mut InstrSeq) {
        for (instr, _) in seq.instrs.iter_mut() {
            if let Instr::AtomicWait(w) = instr
                && let Some(func) = self.helpers.get(&(w.memory, w.arg.offset, w.sixty_four))
            {
                *instr = Instr::Call(Call { func: *func });
                self.replaced += 1;
            }
        }
    }
}

fn helper(
    module: &mut Module,
    page: GlobalId,
    memory: MemoryId,
    arg: MemArg,
    wide: bool,
) -> FunctionId {
    let value = if wide { ValType::I64 } else { ValType::I32 };
    let mut b = FunctionBuilder::new(
        &mut module.types,
        &[ValType::I32, value, ValType::I64],
        &[ValType::I32],
    );
    let addr = module.locals.add(ValType::I32);
    let expected = module.locals.add(value);
    let timeout = module.locals.add(ValType::I64);
    let spun = module.locals.add(ValType::I64);
    let load = match wide {
        true => LoadKind::I64 { atomic: true },
        false => LoadKind::I32 { atomic: true },
    };
    let ne = if wide { BinaryOp::I64Ne } else { BinaryOp::I32Ne };
    let mut body = b.func_body();
    body.global_get(page).unop(walrus::ir::UnaryOp::I32Eqz).if_else(
        None,
        |then| {
            then.local_get(addr)
                .local_get(expected)
                .local_get(timeout)
                .atomic_wait(memory, arg, wide)
                .return_();
        },
        |_| {},
    );
    body.loop_(None, |spin| {
        let again = spin.id();
        spin.local_get(addr).load(memory, load, arg).local_get(expected).binop(ne).if_else(
            None,
            |changed| {
                changed.i32_const(1).return_();
            },
            |_| {},
        );
        spin.local_get(timeout).i64_const(0).binop(BinaryOp::I64GeS).if_else(
            None,
            |bounded| {
                bounded
                    .local_get(spun)
                    .i64_const(1)
                    .binop(BinaryOp::I64Add)
                    .local_tee(spun)
                    .i64_const(4)
                    .binop(BinaryOp::I64Mul)
                    .local_get(timeout)
                    .binop(BinaryOp::I64GtS)
                    .if_else(
                        None,
                        |late| {
                            late.i32_const(2).return_();
                        },
                        |_| {},
                    );
            },
            |_| {},
        );
        spin.br(again);
    });
    body.unreachable();
    b.finish(vec![addr, expected, timeout], &mut module.funcs)
}
