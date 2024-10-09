# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


class TerminalFormatter:
    # Define ANSI color codes
    COLORS = {
        'black': '30',
        'red': '31',
        'green': '32',
        'yellow': '33',
        'blue': '34',
        'magenta': '35',
        'cyan': '36',
        'white': '37',
        'reset': '0',
    }

    BACKGROUNDS = {
        'black': '40',
        'red': '41',
        'green': '42',
        'yellow': '43',
        'blue': '44',
        'magenta': '45',
        'cyan': '46',
        'white': '47',
    }

    # Define text formatting codes
    BOLD = '1'
    ITALIC = '3'
    
    @staticmethod
    def color_text(text, color=None, bg_color=None, bold=False, italic=False):
        # Start with the style codes list
        style_codes = []
        
        # Add text color if provided
        if color and color in TerminalFormatter.COLORS:
            style_codes.append(TerminalFormatter.COLORS[color])
        
        # Add background color if provided
        if bg_color and bg_color in TerminalFormatter.BACKGROUNDS:
            style_codes.append(TerminalFormatter.BACKGROUNDS[bg_color])
        
        # Add formatting options
        if bold:
            style_codes.append(TerminalFormatter.BOLD)
        if italic:
            style_codes.append(TerminalFormatter.ITALIC)
        
        # Join all the style codes into one sequence
        if style_codes:
            style_prefix = f"\033[{';'.join(style_codes)}m"
        else:
            style_prefix = ''
        
        # Reset code
        reset_code = "\033[0m"
        
        # Return the styled text
        return f"{style_prefix}{text}{reset_code}"
# EOF
