from bimanual.models.policy import Policy


class BC(Policy):
    def __init__(self, env, policy_net, **kwargs):
        super().__init__(env)
        self.policy_net = policy_net

    def get_action(self, obs):
        return self.policy_net(obs)
