def string_to_color(str):
    hash = str.__hash__()
    r = ((hash & 0xFF0000) >> 16) / 255.0
    g = ((hash & 0x00FF00) >> 8) / 255.0
    b = (hash & 0x0000FF) / 255.0
    return r, g, b
