import re

def camel_to_snake(camel_str:str)->str:
  name = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', camel_str)
  return re.sub('([a-z0-9])([A-Z])', r'\1_\2', name).lower()

def snake_to_camle(snake_str:str)->str:
    components = snake_str.split('_')
    return components[0] + ''.join(x.title() for x in components[1:])


_ansi_escape = re.compile(r'''
    \x1B    # ESC
    [@-_]   # 7-bit C1 Fe
    [0-?]*  # Parameter bytes
    [ -/]*  # Intermediate bytes
    [@-~]   # Final byte
    ''', re.VERBOSE)

def remove_ansi_escape_code(text):
    return _ansi_escape.sub('', text)

