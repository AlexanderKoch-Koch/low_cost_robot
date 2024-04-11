import os
# fallback to cpu if mps is not available for specific operations
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = "1"
import torch

# data directory
DATA_DIR = 'data/'

# device
device = 'cpu'
if torch.cuda.is_available(): device = 'cuda'
if torch.backends.mps.is_available(): device = 'mps'
os.environ['DEVICE'] = device

# robot port names
ROBOT_PORTS = {
    'leader': '/dev/tty.usbmodem57380045221',
    'follower': '/dev/tty.usbmodem57380046991'
}


# physical task config
TASK_CONFIG = {
    'pick_cube':{
        'dataset_dir': os.path.join(DATA_DIR, 'pick_cube'),
        'num_episodes': 1,
        'episode_len': 150,
        'state_dim': 5,
        'action_dim': 5,
        'cam_width': 640,
        'cam_height': 480,
        'camera_names': ['front']
    }
}


# policy config
POLICY_CONFIG = {
    'lr': 1e-5,
    'device': device,
    'num_queries': 100,
    'kl_weight': 10,
    'hidden_dim': 512,
    'dim_feedforward': 3200,
    'lr_backbone': 1e-5,
    'backbone': 'resnet18',
    'enc_layers': 4,
    'dec_layers': 7,
    'nheads': 8,
    'camera_names': ['front'],
    'policy_class': 'ACT',
    'temporal_agg': False
}

# training config
TRAIN_CONFIG = {
    'seed': 42,
    'num_epochs': 2000,
    'batch_size_val': 8,
    'batch_size_train': 8,
    'eval_ckpt_name': 'policy_last.ckpt',
    'checkpoint_dir': 'checkpoints/pick_cube',
}