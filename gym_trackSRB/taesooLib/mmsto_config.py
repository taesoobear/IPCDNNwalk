from easydict import EasyDict as edict
import math

obs_history_buffer_skip=2
obs_history_buffer_size=21  # when skip==2, this number has to be an odd number to include the last frame.
SRB_obs_dim = 25
#SRB_obs_dim = 16 

fullbody_obs_dim = 202+12
reconstructed_fullbody_obs_dim = fullbody_obs_dim



#model_path = "walkAndrun_test_25.zip"
#model_path = "walkAndrun_test_25.zip"
model_path = "gym_trackSRB/AE_window_continue_new_3.zip"
#obs_history_buffer_size=5 # for predicting poses, dx, ddx, and all the sim-mocap deltas
planningHorizon=7  # mmsto planning horizon. use an odd number. retraining unnecessary after chainging this.
# 0 1 2 3 4 5 6 7 8
# |     mmsto |  latest pose prediction
#   | draw
onlineFilterSize=3  # set 1 to disable
drawDelay=planningHorizon-2+math.floor(onlineFilterSize/2)
inputMotionBufferSize=planningHorizon+1+obs_history_buffer_size
outputMotionBufferSize=planningHorizon+1

# for backward compatibility
mmsto_config=edict(inputMotionBufferSize=inputMotionBufferSize)

#assert(planningHorizon>=obs_history_buffer_size)
assert(inputMotionBufferSize>=planningHorizon+obs_history_buffer_size)


if obs_history_buffer_skip>=2:
    SRB_obs_cols= SRB_obs_dim*(int(obs_history_buffer_size/obs_history_buffer_skip)+1)
else:
    SRB_obs_cols= SRB_obs_dim*obs_history_buffer_size


# 加载训练好的模型
decoder_path = "gym_trackSRB/models_saved_2/FullbodyVAE_decoder_epoch_99999.pth" # 99999 looks better than 199999 even though the loss is a little higher.
encoder_path = "gym_trackSRB/models_saved_2/FullbodyVAE_encoder_epoch_99999.pth" # right balance between underfititng and overfitting is the key.


def loadPosePredictor():
    num_output_poses=planningHorizon-2

    from .FullbodyVAE import FullbodyVAEEncoder, FullbodyVAEDecoder
    import torch
    fullbodyvae_encoder = FullbodyVAEEncoder(SRB_obs_cols, fullbody_obs_dim*num_output_poses)
    fullbodyvae_decoder = FullbodyVAEDecoder(SRB_obs_cols, reconstructed_fullbody_obs_dim*num_output_poses)

    fullbodyvae_encoder.load_state_dict(torch.load(encoder_path, map_location=torch.device('cpu'), weights_only=False))
    fullbodyvae_decoder.load_state_dict(torch.load(decoder_path, map_location=torch.device('cpu'), weights_only=False))

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    fullbodyvae_encoder.to(device)
    fullbodyvae_decoder.to(device)

    # 设置模型为评估模式
    fullbodyvae_encoder.eval()
    fullbodyvae_decoder.eval()


    full_pred=(fullbodyvae_encoder, fullbodyvae_decoder)
    return full_pred, device

