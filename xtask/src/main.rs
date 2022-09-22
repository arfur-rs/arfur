use clap::Parser;
use color_eyre::Result;

mod bindgen;
mod cli;

#[tokio::main]
async fn main() -> Result<()> {
    let cli = cli::Cli::parse();
    cli.exec().await
}
