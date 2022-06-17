class UniqueName:
    def __init__(self, prefix:str=None) -> None:
        self.__names = set()
        self.__prefix = prefix

    def get_name(self, name:str)->str:
        counter = 0
        while True:
            if name not in self.__names:
                self.__names.add(name)
                break
            counter += 1
            name = f"{name}:{counter}"
            if counter > 5:
                raise RuntimeError("Too many same names.")
            
        if self.__prefix is None:
            return name
        else:
            return f"{self.__prefix}:{name}"