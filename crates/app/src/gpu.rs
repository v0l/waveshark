use eframe::egui_wgpu::{WgpuConfiguration, WgpuSetup};
use eframe::wgpu::{Adapter, Backend, DeviceType, Surface};
use std::sync::Arc;

pub fn prefer_dx12_on_windows(options: &mut WgpuConfiguration) {
    if !cfg!(windows) || std::env::var_os("WGPU_BACKEND").is_some() {
        return;
    }
    if let WgpuSetup::CreateNew(setup) = &mut options.wgpu_setup {
        setup.native_adapter_selector = Some(Arc::new(pick_adapter));
    }
}

fn pick_adapter(adapters: &[Adapter], surface: Option<&Surface<'_>>) -> Result<Adapter, String> {
    adapters
        .iter()
        .filter(|a| surface.is_none_or(|s| a.is_surface_supported(s)))
        .min_by_key(|a| {
            let info = a.get_info();
            rank(&(info.backend, info.device_type))
        })
        .cloned()
        .ok_or_else(|| "no graphics adapter can draw to this window".to_string())
}

fn rank(&(backend, device_type): &(Backend, DeviceType)) -> (bool, u8) {
    let device = match device_type {
        DeviceType::DiscreteGpu => 0,
        DeviceType::IntegratedGpu => 1,
        DeviceType::VirtualGpu => 2,
        DeviceType::Other => 3,
        DeviceType::Cpu => 4,
    };
    (backend != Backend::Dx12, device)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dx12_is_chosen_over_vulkan_on_the_same_card() {
        let mut found = [
            (Backend::Vulkan, DeviceType::DiscreteGpu),
            (Backend::Dx12, DeviceType::DiscreteGpu),
            (Backend::Vulkan, DeviceType::IntegratedGpu),
            (Backend::Dx12, DeviceType::IntegratedGpu),
        ];
        found.sort_by_key(rank);
        assert_eq!(
            found,
            [
                (Backend::Dx12, DeviceType::DiscreteGpu),
                (Backend::Dx12, DeviceType::IntegratedGpu),
                (Backend::Vulkan, DeviceType::DiscreteGpu),
                (Backend::Vulkan, DeviceType::IntegratedGpu),
            ]
        );
    }

    #[test]
    fn vulkan_on_the_discrete_card_where_there_is_no_dx12() {
        let best = [
            (Backend::Vulkan, DeviceType::IntegratedGpu),
            (Backend::Vulkan, DeviceType::DiscreteGpu),
        ]
        .into_iter()
        .min_by_key(rank)
        .unwrap();
        assert_eq!(best, (Backend::Vulkan, DeviceType::DiscreteGpu));
    }
}
