use clap::Args;
use color_eyre::Result;
use tracing_subscriber::prelude::*;
use tracing_subscriber::{fmt, EnvFilter};

#[derive(Args, Debug)]
pub struct BindgenArgs {
    #[arg(short, long)]
    verbose: bool,
}

impl BindgenArgs {
    pub async fn exec(self) -> Result<()> {
        let fmt_layer = fmt::layer().with_target(false);

        let filter_layer = if self.verbose {
            EnvFilter::try_from_default_env()
                .or_else(|_| EnvFilter::try_new("trace"))
                .unwrap()
        } else {
            EnvFilter::try_from_default_env()
                .or_else(|_| EnvFilter::try_new("info"))
                .unwrap()
        };

        tracing_subscriber::registry()
            .with(filter_layer)
            .with(fmt_layer)
            .init();

        arfur_build::runners::wpilib::run(false).await?;
        arfur_build::runners::rev::run(false).await?;

        Ok(())
    }
}
