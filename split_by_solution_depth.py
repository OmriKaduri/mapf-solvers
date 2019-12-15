import sys
import os.path
import csv

from utils import keydefaultdict

input_path = sys.argv[1]
input_filename = os.path.splitext(input_path)[0]

reader = csv.DictReader(open(input_path))


def make_csv_writer(solution_depth):
    'Uses input_filename and reader.fieldnames globals'
    path = "%s_depth%s.csv" % (input_filename, solution_depth)
    writer = csv.DictWriter(open(path, 'wb'), reader.fieldnames)
    writer.writeheader()
    return writer


gridname_to_csv_writer = keydefaultdict(make_csv_writer)
solution_depth_columns = [col_name for col_name in reader.fieldnames if col_name.endswith(" Solution Depth")]

for row in reader:
    for col_name in solution_depth_columns:
        depth = int(row[col_name])
        if depth != -1:
            gridname_to_csv_writer[depth].writerow(row)
            break
    else:
        gridname_to_csv_writer[-1].writerow(row)
