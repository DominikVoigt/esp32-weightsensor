class Weight_DTO:


    def __init__(self):
        print("Initialize Weigth DTO")
    
    def __init__(self, name:str, weight:float):
        self.name = name
        self.weight = weight
        self.full = None
        self.empty = None