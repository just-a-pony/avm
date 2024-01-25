// TODO(comc): Remove non-wasm code paths.
use re_memory::AccountingAllocator;

#[global_allocator]
static GLOBAL: AccountingAllocator<std::alloc::System> = AccountingAllocator::new(std::alloc::System);

#[cfg(target_family = "wasm")]
fn main() {
    eframe::WebLogger::init(log::LevelFilter::Debug).ok();
    let web_options = eframe::WebOptions::default();

    wasm_bindgen_futures::spawn_local(async {
        eframe::WebRunner::new()
            .start(
                "avm_analyzer_canvas_id",
                web_options,
                Box::new(|cc| {
                    egui_extras::install_image_loaders(&cc.egui_ctx);
                    Box::new(avm_analyzer_app::AvmAnalyzerApp::new(cc))
                }),
            )
            .await
            .expect("failed to start eframe");
    });
}
