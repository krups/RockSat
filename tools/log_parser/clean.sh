# directory paths
decompress_dir=./decompressed_packets
output_dir=./parser_output

# utility binary paths
decompress=./decompressor
parser=./log_parser

# If directory exists, remove it and its contents
[ -d $decompress_dir ] && rm -rf $decompress_dir && echo "Removing $decompress_dir and its contents"
[ -d $output_dir ] && rm -rf $output_dir && echo "Removing $output_dir and its contents"

# If util binaries exists, remove them
[ -f $decompress ] && rm -f $decompress && echo "Removing $decompress"
[ -f $parser ] && rm -f $parser && echo "Removing $parser"
echo "Cleanup finished"