Log parser for Iriduium Packets 

Directories and files:

- log_parser
    - decompress
        - brieflz.c
        - brieflz.h
        - depacks.c
        - main.cpp
    - parser
        - config.h
        - log_parser.cpp
        - packet.h
    - batch_parse.sh
    - clean.sh
    - display_logs.py
    - README.md

Utilities:

----- batch_parse.sh -----
- Compiles decompress and log_parser programs
- Passes all files in raw_packets directory into decompressor and log_parser and stores output in parser_output directory
- starts display_logs.py

----- clean.sh -----
- Removes temporary folders and files created by batch_parse.sh

----- display_logs.py -----
- Reads files in from parser_output directory
- Creates CSV sorted by time
- Creates graphs to display data
- Usage:
    - Displays data and creates csv from parser output
    - `python ./display_logs.py` 

    - Imports and displays data from existing csv
    - `python ./display_logs.py <csv file name>`

To use:
- Store packets from Iridium in raw_packets directory (create if doesn't exist)
- Run batch_parse.sh
- If parser_outputs exist, can run display_logs.py