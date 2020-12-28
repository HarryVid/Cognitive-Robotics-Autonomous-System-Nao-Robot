# Nao motion library


def bot_move_forward_backend(motion,value):
    motion.post.moveTo(value,0,0)

def bot_move_backward_backend(motion,value):
    motion.post.moveTo(-value,0,0)
    
def bot_move_right_backend(motion,value):
    motion.post.moveTo(0,0,value)

def bot_move_left_backend(motion,value):
    motion.post.moveTo(0,0,-value)
    
def bot_move_forward(motion,value):
    motion.moveTo(value,0,0)

def bot_move_backward(motion,value):
    motion.moveTo(-value,0,0)
    
def bot_move_right(motion,value):
    motion.moveTo(0,0,-value)

def bot_move_left(motion,value):
    motion.moveTo(0,0,value)
