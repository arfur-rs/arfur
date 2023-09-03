use clap::Args;
use color_eyre::Result;

#[derive(Args, Debug)]
pub struct BindgenArgs {
  #[arg(short, long)]
  verbose: bool,
}

impl BindgenArgs {
    pub async fn exec(self) -> Result<()> {
        if true {
            let subscriber = tracing_subscriber::fmt().compact().finish();
            tracing::subscriber::set_global_default(subscriber)?;
        }

        arfur_build::runners::wpilib::run(false).await?;
        arfur_build::runners::rev::run(false).await?;

        Ok(())
    }
}
