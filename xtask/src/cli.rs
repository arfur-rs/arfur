use clap::{Parser, Subcommand};
use color_eyre::Result;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
#[clap(propagate_version = true)]
pub struct Cli {
    #[clap(subcommand)]
    pub command: Command,
}

#[derive(Subcommand, Debug)]
#[clap(bin_name = "cargo")]
pub enum Command {
    Bindgen(crate::bindgen::BindgenArgs),
}

impl Cli {
    pub async fn exec(self) -> Result<()> {
        match self.command {
            Command::Bindgen(x) => x.exec().await,
        }
    }
}
