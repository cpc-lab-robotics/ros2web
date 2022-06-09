from hashids import Hashids


class KeyGenerator:
    def __init__(self, *, salt, min_length=6) -> None:
        
        self.__hashids = Hashids(
            salt=salt,
            min_length=min_length
        )
        self.__key_counter = 0

    def get_key(self):
        key = self.__hashids.encode(self.__key_counter)
        self.__key_counter += 1
        return key
