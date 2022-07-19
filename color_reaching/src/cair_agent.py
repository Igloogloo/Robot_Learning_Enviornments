from sac_node import Agent
from PS_utils import sample_normal

class CairAgent():
    def __init__(self, env_alpha=0.001, env_beta=0.001, teach_alpha=0.001, teach_beta=0.001,
            input_dims=[8], env=None, gamma=0.99, n_actions=2, env_max_size=1000000, teach_max_size=100000,
            env_tau=0.005, teach_tau=0.005, layer1_size=256, layer2_size=256, env_batch_size=256, 
            teach_batch_size=256, env_reward_scale=2, teach_reward_scale=10, env_auto_entropy=False, 
            teach_auto_entropy=False, env_entr_lr=None, teach_entr_lr=None, reparam_noise=1e-6, max_action=1, kappa=.9):

        self.env_agent = Agent(alpha=env_alpha, beta=env_beta, input_dims=input_dims, env=env, max_size=env_max_size, 
            tau=env_tau, layer1_size=layer1_size, layer2_size=layer2_size, gamma=gamma, n_actions=n_actions, 
            batch_size=env_batch_size, reward_scale=env_reward_scale, auto_entropy=env_auto_entropy, 
            entr_lr=env_entr_lr, reparam_noise=reparam_noise, max_action=max_action)

        self.teach_agent = Agent(alpha=teach_alpha, beta=teach_beta, input_dims=input_dims, env=env, 
            max_size=teach_max_size, tau=teach_tau, layer1_size=layer1_size, layer2_size=layer2_size, gamma=gamma, 
            n_actions=n_actions, batch_size=teach_batch_size, reward_scale=teach_reward_scale, 
            auto_entropy=teach_auto_entropy, entr_lr=teach_entr_lr, reparam_noise=reparam_noise, max_action=max_action)
        
        self.kappa = kappa

    def get_action(self, observation, with_noise=False, max_action=1, env_only=False, 
        with_grad_env=True, with_grad_agent=True, kappa=None):
        
        if kappa is None:
            action, dist, mu, sigma = sample_normal(agent=self.teach_agent, actor=self.env_agent, 
            observation=observation,with_noise=with_noise, max_action=max_action, env_only=env_only,
            with_grad_env=with_grad_env, with_grad_agent=with_grad_agent, kappa=self.kappa)
            
            return action, dist, mu, sigma

        else:
            action, dist, mu, sigma = sample_normal(agent=self.teach_agent, actor=self.env_agent, 
            observation=observation,with_noise=with_noise, max_action=max_action, env_only=env_only,
            with_grad_env=with_grad_env, with_grad_agent=with_grad_agent, kappa=kappa)
            
            return action, dist, mu, sigma

            
