#!/usr/bin/env python3
"""Dump what the reference Kokoro implementation computes, stage by stage.

The Rust port in `crates/tts/src/kokoro` is checked against this:
`cargo run --release -p tts --example reference` reads the safetensors this
writes and prints the error at every stage.

Needs torch, transformers, huggingface_hub, safetensors and a clone of
https://github.com/hexgrad/kokoro whose `kokoro/` directory is importable as
`kok` (copy it and empty its `__init__.py`, which otherwise pulls in the
espeak front end).

    python3 tools/kokoro_reference.py /tmp/kokoro_ref.safetensors
"""

import sys

import torch
from huggingface_hub import hf_hub_download
from safetensors.torch import save_file

from kok.model import KModel

PHONEMES = "hɛlˈO wˌɜɹld"
VOICE = "af_heart"


def main(out_path):
    model = KModel(repo_id="hexgrad/Kokoro-82M").eval()
    voice = torch.load(
        hf_hub_download("hexgrad/Kokoro-82M", f"voices/{VOICE}.pt"), weights_only=True
    )
    ids = [0] + [model.vocab[p] for p in PHONEMES if p in model.vocab] + [0]
    input_ids = torch.LongTensor([ids])
    style = voice[len(ids) - 2]
    out = {"input_ids": input_ids.float(), "ref_s": style}

    with torch.no_grad():
        lengths = torch.full((1,), len(ids), dtype=torch.long)
        mask = torch.arange(len(ids)).unsqueeze(0) + 1 > lengths.unsqueeze(1)
        out["bert"] = model.bert(input_ids, attention_mask=(~mask).int())
        out["d_en"] = model.bert_encoder(out["bert"]).transpose(-1, -2)

        spoken = style[:, 128:]
        out["d"] = model.predictor.text_encoder(out["d_en"], spoken, lengths, mask)
        lstm, _ = model.predictor.lstm(out["d"])
        durations = torch.sigmoid(model.predictor.duration_proj(lstm)).sum(axis=-1)
        durations = torch.round(durations).clamp(min=1).long().squeeze()
        out["dur"] = durations.float()

        index = torch.repeat_interleave(torch.arange(input_ids.shape[1]), durations)
        aligned = torch.zeros((input_ids.shape[1], index.shape[0]))
        aligned[index, torch.arange(index.shape[0])] = 1
        aligned = aligned.unsqueeze(0)
        out["en"] = out["d"].transpose(-1, -2) @ aligned

        shared, _ = model.predictor.shared(out["en"].transpose(-1, -2))
        out["shared"] = shared
        block = shared.transpose(-1, -2)
        for i, layer in enumerate(model.predictor.F0):
            block = layer(block, spoken)
            out[f"F0blk{i}"] = block
        out["F0"], out["N"] = model.predictor.F0Ntrain(out["en"], spoken)

        out["t_en"] = model.text_encoder(input_ids, lengths, mask)
        out["asr"] = out["t_en"] @ aligned
        out["audio"] = decoder_stages(model, out, style[:, :128])

    save_file({k: v.detach().clone().contiguous().float() for k, v in out.items()}, out_path)
    print({k: tuple(v.shape) for k, v in out.items()})


def decoder_stages(model, out, voiced):
    """Run the vocoder a stage at a time, keeping each one."""
    import torch.nn.functional as F

    torch.manual_seed(0)
    decoder = model.decoder
    out["F0h"] = decoder.F0_conv(out["F0"].unsqueeze(1))
    out["Nh"] = decoder.N_conv(out["N"].unsqueeze(1))
    x = torch.cat([out["asr"], out["F0h"], out["Nh"]], axis=1)
    x = decoder.encode(x, voiced)
    out["enc"] = x
    out["asr_res"] = decoder.asr_res(out["asr"])
    for i, block in enumerate(decoder.decode):
        x = torch.cat([x, out["asr_res"], out["F0h"], out["Nh"]], axis=1)
        x = block(x, voiced)
        out[f"dec{i}"] = x

    gen = decoder.generator
    f0 = gen.f0_upsamp(out["F0"][:, None]).transpose(1, 2)
    har_source, _, _ = gen.m_source(f0)
    har_source = har_source.transpose(1, 2).squeeze(1)
    out["har_source"] = har_source
    spec, phase = gen.stft.transform(har_source)
    har = torch.cat([spec, phase], dim=1)
    out["har"] = har
    for i in range(gen.num_upsamples):
        x = F.leaky_relu(x, negative_slope=0.1)
        source = gen.noise_res[i](gen.noise_convs[i](har), voiced)
        x = gen.ups[i](x)
        if i == gen.num_upsamples - 1:
            x = gen.reflection_pad(x)
        x = x + source
        total = None
        for j in range(gen.num_kernels):
            one = gen.resblocks[i * gen.num_kernels + j](x, voiced)
            total = one if total is None else total + one
        x = total / gen.num_kernels
        out[f"up{i}"] = x
    x = gen.conv_post(F.leaky_relu(x))
    out["post"] = x
    out["spec"] = torch.exp(x[:, :11, :])
    out["phase"] = torch.sin(x[:, 11:, :])
    return gen.stft.inverse(out["spec"], out["phase"]).squeeze()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/kokoro_ref.safetensors")
