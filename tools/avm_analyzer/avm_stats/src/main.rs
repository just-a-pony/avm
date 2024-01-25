use avm_stats::Frame;
use clap::Parser;
use prost::Message;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to protobuf file.
    #[arg(short, long)]
    proto: String,
}

fn main() -> Result<(), anyhow::Error> {
    let args = Args::parse();
    let frame = std::fs::read(args.proto).unwrap();
    let frame = Frame::decode(frame.as_slice()).unwrap();
    println!("{:?}", frame.superblocks.len());
    Ok(())
}
