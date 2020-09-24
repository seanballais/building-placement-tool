import importlib
import sys


def display_help():
    print('CoreX Tool Runner')
    print('-----------------')
    print('Runs CoreX tools.')
    print('')
    print('Usage:')
    print('  run_tools.py <tool name> (tool arguments)...')
    print('')


def main(args):
    tools_dir = 'corex/tools/'
    sys.path.append(tools_dir)

    tool_name = args[0]
    tool_args = args[1:] if len(args) > 1 else []

    tool = importlib.import_module(tool_name)
    tool_max_args = tool.MAX_ARGS

    if (('--help' in tool_args or '-h' in tool_args)
            or len(tool_args) != tool_max_args):
        tool.display_help()
    else:
        tool.run(tool_args)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        display_help()
    else:
        main(sys.argv[1:])
