#!/usr/bin/env python3

def hex_to_rgb(hex, ansi = True):
    stripped_hex = hex.lstrip('#')
    r = int(stripped_hex[0:2], 16)
    g = int(stripped_hex[2:4], 16)
    b = int(stripped_hex[4:6], 16)
    if ansi:
        return f'{r};{g};{b}'
    else:
        return tuple(r, g, b)

def rgb_to_hex(rgb):
    return f'#{rgb[0]:x}{rgb[1]:x}{rgb[2]:x}'.upper()

def gen_esc(code):
    return f'\033[{code}m'

def gen_ansi_rgb_esc(code):
    return f'\033[38;2;{code}m'

"""
sequences = {
    'EFFECTS' : {
        'RESET'     : gen_esc(0),
        'BOLD'      : gen_esc(1),
        'ITALIC'    : gen_esc(3),
        'UNDERLINE' : gen_esc(4)
    },

    'FLAGS' : {
        'ERROR'     : gen_esc(91),
        'SUCCESS'   : gen_esc(92),
        'WARNING'   : gen_esc(93),
        'LOG'       : gen_esc(0)    # basically reset?
    },

    # example of adding custom color scheme if your terminal isn't under a scheme already
    'DEFAULT' : {
        'RED'       : gen_ansi_rgb_esc(hex_to_rgb('#ff0000')),
        'GREEN'     : gen_ansi_rgb_esc(hex_to_rgb('#00ff00')),
        'BLUE'      : gen_ansi_rgb_esc(hex_to_rgb('#0000ff')),
        'CYAN'      : gen_ansi_rgb_esc(hex_to_rgb('#00ffff')),
        'MAGENTA'   : gen_ansi_rgb_esc(hex_to_rgb('#ff00ff')),
        'YELLOW'    : gen_ansi_rgb_esc(hex_to_rgb('#ffff00')),
        'WHITE'     : gen_ansi_rgb_esc(hex_to_rgb('#ffffff')),
        'BLACK'     : gen_ansi_rgb_esc(hex_to_rgb('#000000')),
    },

    # dracula: https://draculatheme.com/contribute#color-palette
    'DRACULA' : {
        'CYAN'      : gen_ansi_rgb_esc(hex_to_rgb('#8be9fd')),
        'GREEN'     : gen_ansi_rgb_esc(hex_to_rgb('#50fa7b')),
        'ORANGE'    : gen_ansi_rgb_esc(hex_to_rgb('#ffb86c')),
        'PINK'      : gen_ansi_rgb_esc(hex_to_rgb('#ff79c6')),
        'PURPLE'    : gen_ansi_rgb_esc(hex_to_rgb('#bd93f9')),
        'RED'       : gen_ansi_rgb_esc(hex_to_rgb('#ff5555')),
        'YELLOW'    : gen_ansi_rgb_esc(hex_to_rgb('#f1fa8c'))
    }
}
"""

class ColorLogger:
    def __init__(self, label = None):
        self.prepend = str('') if label == None else f'[{label}] '
        self.define_sequences()

    def gen_output_str(self, msg, esc, indent = 0):
        return esc + self.prepend + (' ' * indent * 4) + str(msg) + self.reset

    def log_info(self, msg, indent = 0):
        print(self.gen_output_str(msg, self.reset, indent=indent))

    def log_warn(self, msg, indent = 0):
        print(self.gen_output_str(msg, self.warn_esc, indent=indent))

    def log_success(self, msg, indent = 0):
        print(self.gen_output_str(msg, self.success_esc, indent=indent))

    def log_error(self, msg, indent = 0):
        print(self.gen_output_str(msg, self.error_esc, indent=indent))

    def define_sequences(self):
        self.reset = gen_esc(0)
        self.error_esc = gen_esc(91)
        self.success_esc = gen_esc(92)
        self.warn_esc = gen_esc(93)