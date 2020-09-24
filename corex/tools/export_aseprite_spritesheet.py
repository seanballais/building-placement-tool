import json
import os
import shutil
import statistics
import sys


MAX_ARGS = 2


def display_help():
    print('CoreX Export Aseprite Spritesheet as CoreX Spritesheet Data')
    print('--------------------------------------------------------------')
    print('Exports an Aseprite Spritesheet JSON file to a CoreX Spritesheet '
          'data file in a user-specified folder. Note that this will also '
          'copy the spritesheet associated to the destination folder.')
    print('')
    print('Usage:')
    print('  export_aseprite_spritesheet.py <source file> '
          '<destination folder>')
    print('')

def run(args):
    source_file = args[0]
    dest_folder = args[1]

    print('Exporting Aseprite Spritesheet to CoreX Spritesheet Data...')
    print('--------------------------------------------------------------')
    print(f'Source:\t\t{source_file}')
    print(f'Destination:\t{dest_folder}')
    print('--------------------------------------------------------------')

    with open(source_file) as data_file:
        aseprite_data = json.load(data_file)

    spritesheet_data = dict()
    
    # Make sure the frames are in the right order.
    aseprite_frames = aseprite_data['frames']
    frames = list()
    frame_durations = list()
    frame_keys = sorted(aseprite_frames.keys())
    for key in frame_keys:
        curr_frame = aseprite_frames[key]
        frame_data = curr_frame['frame']

        frames.append(frame_data)
        frame_durations.append(curr_frame['duration'])

    spritesheet_data['frames'] = frames

    possible_frame_durations = statistics.multimode(frame_durations)
    if len(possible_frame_durations) == 1:
        duration = possible_frame_durations[0]
    else:
        print('There are multiple durations in the '
              'Aseprite spritesheet data. ', end='')
        while True:
            print('Select a frame duration:')
            for item_num, duration in enumerate(possible_frame_durations, 1):
                print(f'  {item_num}) {duration}ms')
            
            selection = input('Option (type the item number): ')

            if (selection.isnumeric()
                    and 1 <= int(selection) <= len(possible_frame_durations)):
                duration = possible_frame_durations[selection - 1]
                break
            else:
                print('Invalid option. Please try again.')

    spritesheet_data['time_per_frame'] = duration / 1000

    aseprite_tags = aseprite_data['meta']['frameTags']
    tags = list()
    for tag in aseprite_tags:
        tags.append({
            'name': tag['name'],
            'from': tag['from'],
            'to': tag['to']
        })

    spritesheet_data['custom_states'] = tags

    data_json = json.dumps(spritesheet_data)

    # For POSIX compliance. View more information on why we add an
    # additional newline here:
    #   https://stackoverflow.com/a/729795/1116098
    #
    # TL;DR: Taking from the link above, it's "because that’s how the
    #        POSIX standard defines a line."
    data_json += '\n'

    source_file_parts = source_file.split('/')
    source_folder = '/'.join(source_file_parts[:len(source_file_parts) - 1])

    source_file_name = source_file_parts[len(source_file_parts) - 1]
    source_file_name = source_file_name.split('.')[0]

    spritesheet_filename = f'{source_file_name}.png'

    source_spritesheet_file = os.path.join(source_folder, spritesheet_filename)

    dest_filename = f'{source_file_name}.cxssd'
    dest_data_file = os.path.join(dest_folder, dest_filename)
    dest_spritesheet_file = os.path.join(dest_folder, spritesheet_filename)

    with open(dest_data_file, 'w') as spritesheet_data_file:
        spritesheet_data_file.write(data_json)

    shutil.copy(source_spritesheet_file, dest_spritesheet_file)

    print('Export completed.')

if __name__ == '__main__':
    if ('--help' in sys.argv or '-h' in sys.argv) or len(sys.argv) != 3:
        display_help()
    else:
        run(sys.argv[1:])
