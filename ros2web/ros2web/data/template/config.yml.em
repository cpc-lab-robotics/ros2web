hot_reload:
  patterns: ["*.py", "*.yml"]
@[if extension]@
process_disable: True
Widget:
  grid:
    w: 3
    h: 3
    # minW: 3
    # minH: 3
    # maxW: 3
    # maxH: 3
  props:
    label: Hello
@[end if]@
@[if package]@
page:
  style: Grid
  widgets:
    - name: std.Button
      id: button
      props:
        label: ${button_label}
@[end if]@