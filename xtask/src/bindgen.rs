use clap::Args;
use color_eyre::Result;

#[derive(Args, Debug)]
pub struct BindgenArgs {}

impl BindgenArgs {
    pub async fn exec(self) -> Result<()> {
        arfur_build::runners::wpilib::run().await?;

        Ok(())
    }
}
