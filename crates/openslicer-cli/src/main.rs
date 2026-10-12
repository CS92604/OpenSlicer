fn main() {
    let args: Vec<String> = std::env::args().collect();
    let code = openslicer_cli::run(args, &mut std::io::stdout(), &mut std::io::stderr());
    std::process::exit(code);
}
