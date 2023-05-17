class Agent:
    def __init__(self,
                 obs_type : str) -> None:
        
        self.obs_type = obs_type
        self.init_policy_nets()
        

    def init_policy_nets(self):
        raise NotImplementedError
    
    def train(self):
        raise NotImplementedError
    
    def eval(self):
        raise NotImplementedError
    
    def save(self):
        raise NotImplementedError
