#!/usr/bin/env python

def read_from_file():
    # Nome del file da leggere
    nome_file = "map_file.txt"

    # Open the text file
    with open(nome_file, 'r') as file:
        lines = file.readlines()
    
    map_data = []
    # Iterate through each line in the file
    for index, line in enumerate(lines):
        # Strip leading and trailing whitespace from the line
        line = line.strip()

        # Extract map data from the first line
        if index == 0:
            if line.startswith("map"):
                map_data = eval(line.split('=')[1].strip())
                continue  # Move to the next line

    return map_data