# directory paths
decompress_dir=./decompressed_packets
output_dir=./parser_output

# utility binary paths
decompress=./decompressor
parser=./log_parser

# If directory exists, remove it and its contents
[ -d $decompress_dir ] && rm -rf $decompress_dir
[ -d $output_dir ] && rm -rf $output_dir

# If util binaries exists, remove them
[ -f $decompress ] && rm -f $decompress
[ -f $parser ] && rm -f $parser
