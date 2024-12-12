import requests

def get_next_move(fen):
    header = {fen}
    r = requests.post("https://chess-api.com/v1", json=header)
    return r.text  # modify so it only returns the next move
