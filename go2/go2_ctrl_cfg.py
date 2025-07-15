unitree_go2_flat_cfg = {
    'seed': 42,
    'device': 'cuda:0',
    'num_steps_per_env': 24,
    'max_iterations': 1500,
    'empirical_normalization': False,
    'policy': {
        'class_name': 'ActorCritic',
        'init_noise_std': 1.0,
        'actor_hidden_dims': [128, 128, 128],
        'critic_hidden_dims': [128, 128, 128],
        'activation': 'elu'
    },
    'algorithm': {
        'class_name': 'PPO',
        'value_loss_coef': 1.0,
        'use_clipped_value_loss': True,
        'clip_param': 0.2,
        'entropy_coef': 0.01,
        'num_learning_epochs': 5,
        'num_mini_batches': 4,
        'learning_rate': 0.001,
        'schedule': 'adaptive',
        'gamma': 0.99,
        'lam': 0.95,
        'desired_kl': 0.01,
        'max_grad_norm': 1.0
    },
    'save_interval': 50,
    'experiment_name': 'unitree_go2_flat',
    'run_name': '',
    'logger': 'tensorboard',
    'neptune_project': 'isaaclab',
    'wandb_project': 'isaaclab',
    'resume': False,
    'load_run': 'unitree_go2',
    'load_checkpoint': 'flat_model_6800.pt'
}

unitree_go2_rough_cfg = {
        'seed': 42, 
        'device': 'cuda', 
        'num_steps_per_env': 24, 
        'max_iterations': 15000, 
        'empirical_normalization': False, 
        'policy': {
            'class_name': 'ActorCritic', 
            'init_noise_std': 1.0, 
            'actor_hidden_dims': [512, 256, 128], 
            'critic_hidden_dims': [512, 256, 128], 
            'activation': 'elu'
            }, 
        'algorithm': {
            'class_name': 'PPO', 
            'value_loss_coef': 1.0, 
            'use_clipped_value_loss': True, 
            'clip_param': 0.2, 
            'entropy_coef': 0.01, 
            'num_learning_epochs': 5, 
            'num_mini_batches': 4, 
            'learning_rate': 0.001, 
            'schedule': 'adaptive', 
            'gamma': 0.99, 
            'lam': 0.95, 
            'desired_kl': 0.01, 
            'max_grad_norm': 1.0
        }, 
        'save_interval': 50, 
        'experiment_name': 'unitree_go2_rough', 
        'run_name': '', 
        'logger': 'tensorboard', 
        'neptune_project': 'orbit', 
        'wandb_project': 'orbit', 
        'resume': False, 
        'load_run': 'unitree_go2', 
        'load_checkpoint': 'rough_model_7850.pt'
}
