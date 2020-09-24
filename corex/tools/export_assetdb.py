from collections import OrderedDict
import json
import os
import sys

import openpyxl


MAX_ARGS = 2


def display_help():
    print('CoreX Export as CoreX Asset DB')
    print('----------------------------------------------------------')
    print('Exports a specially formatted XLSX file to assets.cxadb in a '
          'user-specified folder.')
    print('')
    print('Usage:')
    print('  export_assetdb.py <source file> <destination folder>')
    print('')


def run(args):
    source_file = args[0]
    dest_folder = args[1]

    print('Exporting XLSX file to CoreX Asset DB...')
    print('-----------------------------------------------------------')
    print(f'Source:\t\t{source_file}')
    print(f'Destination:\t{dest_folder}')
    print('-----------------------------------------------------------')

    # XLSX file format must be:
    # --------------------------
    # | Asset ID | Type | File |
    # |----------|------|------|
    # |   ...    |  ... | ...  |
    # First row will always be ignored, and is assumed to contain the
    # header.
    assetDB_wb = openpyxl.load_workbook(source_file)
    data_ws = assetDB_wb.active
    num_rows = data_ws.max_row

    print(f'Exporting from XLSX file with {num_rows - 1} entries.')

    db_data = list()
    seen_asset_ids = set()
    seen_asset_files = set()
    for row in range(2, num_rows + 1):  # We're 1-indexed here.
        # We start with the second row, since the first row is
        # reserved for the header.
        asset_id = str(data_ws[f'A{row}'].value)
        asset_type = str(data_ws[f'B{row}'].value)
        asset_file = str(data_ws[f'C{row}'].value)

        # Skip over a row when it has an asset ID or file that is already
        # present in another row.
        if asset_id in seen_asset_ids or asset_file in seen_asset_files:
            if asset_id in seen_asset_ids:
                print(f'WARNING: Row with asset id, "{asset_id}", is '
                       'already used in another row. Skipping over '
                      f'current row (row {row}).')

            if asset_file in seen_asset_files:
                print(f'WARNING: Row with asset file, "{asset_file}", is '
                       'already used in another row. Skipping over '
                      f'current row (row {row}).')
            
            continue

        db_data.append(
            OrderedDict([
                ( 'id', asset_id, ),
                ( 'type', asset_type, ),
                ( 'file', asset_file, )
            ])
        )

        seen_asset_ids.add(asset_id)
        seen_asset_files.add(asset_file)

    # Sort list by type to help with branch prediction when CoreX is
    # parsing the asset database.
    db_data.sort(key=lambda a: (a['type'], a['id'], a['file']))

    data_json = json.dumps(db_data)

    # For POSIX compliance. View more information on why we add an
    # additional newline here:
    #   https://stackoverflow.com/a/729795/1116098
    #
    # TL;DR: Taking from the link above, it's "because that’s how the
    #        POSIX standard defines a line."
    data_json += '\n'

    # Save the data into binary and compress it. We don't want a huge asset
    # database file, don't we?

    dest_filename = 'assets.cxadb'
    dest = os.path.join(dest_folder, dest_filename)
    with open(dest, 'w') as asset_db:
        asset_db.write(data_json)

    print("Export completed.")


if __name__ == '__main__':
    if ('--help' in sys.argv or '-h' in sys.argv) or len(sys.argv) != 3:
        display_help()
    else:
        run(sys.argv[1:])