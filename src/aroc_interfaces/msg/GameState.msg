# Referee state as the behavior layer needs it. Filled by the GameController
# bridge (RoboCup HSL protocol v20); when nobody publishes it the behavior
# falls back to the manual start/stop button.
uint8 INITIAL=0
uint8 READY=1
uint8 SET=2
uint8 PLAYING=3
uint8 FINISHED=4

uint8 SET_PLAY_NONE=0
uint8 SET_PLAY_DIRECT_FREE_KICK=1
uint8 SET_PLAY_INDIRECT_FREE_KICK=2
uint8 SET_PLAY_PENALTY_KICK=3
uint8 SET_PLAY_THROW_IN=4
uint8 SET_PLAY_GOAL_KICK=5
uint8 SET_PLAY_CORNER_KICK=6

uint8 PHASE_NORMAL=0
uint8 PHASE_PENALTY_SHOOT_OUT=1
uint8 PHASE_EXTRA_TIME=2
uint8 PHASE_TIMEOUT=3

std_msgs/Header header
uint8 state
uint8 game_phase
uint8 set_play
bool stopped              # referee pressed "stop play": nobody may move
bool first_half
int16 secs_remaining
int16 secondary_time      # e.g. seconds until the ball is free after a kick-off

bool kicking_team_valid   # false when no team has a kick-off / set play
bool kickoff_ours         # true when our team takes the kick-off / set play

bool penalized            # this robot is penalized
uint8 penalty             # raw PENALTY_* code from the GameController
uint8 secs_till_unpenalized
bool is_goalkeeper

uint8 own_score
uint8 opponent_score
bool from_game_controller
