# removes previous temp data if it exists
./clean.sh

# directory paths
raw_dir=./raw_packets # change to directory of raw packets
decompress_dir=./decompressed_packets
output_dir=./parser_output
csv_dir=./csv_bin

# utility binary paths
decompress=./decompressor
parser=./log_parser

# check for directories and create if they don't exist
[ ! -d $raw_dir ] && mkdir $raw_dir
[ ! -d $decompress_dir ] && mkdir $decompress_dir
[ ! -d $output_dir ] && mkdir $output_dir
[ ! -d $csv_dir ] && mkdir $csv_dir


# check for util binaries and compile if they don't exist
[ ! -f $parser ] && g++ ./parser/log_parser.cpp -o log_parser
[ ! -f $decompress ] && g++ ./decompress/brieflz.c ./decompress/depacks.c ./decompress/main.cpp -o decompressor

# Decompress raw packets
count=1
for x in "$raw_dir"/*; do
    echo "---------------------------------------------------------------"
    echo "Decompressing file $x: $count"
    $decompress $x "$decompress_dir"/decompressed_$count
    ((count++))
done
echo "-------------------------------------------------------------------------------"

# Parse decompressed packets and store outputs
count=1
for x in ./decompressed_packets/*; do
    echo "Writing logs of $x to $output_dir/out_$count"
    $parser packet $x >$output_dir/out_$count
    ((count++))
done
echo "-------------------------------------------------------------------------------"

echo "Running display_logs.py"
# python3 ./display_logs.py
