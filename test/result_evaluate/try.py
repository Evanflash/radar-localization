import math

def get_sin_sum(theta):
    sum = 0
    for i in range(1, 400):
        sum = sum + math.sin(i * theta / 400)
    return sum

def get_cos_sum(theta):
    sum = 0
    for i in range(1, 400):
        sum = sum + math.cos(i * theta / 400)
    return sum

if __name__ == "__main__":
    theta = 0.115846
    x = 0.15087
    y = 1.020435
    vx = x / get_sin_sum(theta) * 400 / 0.25
    vy = y / get_cos_sum(theta) * 400 / 0.25
    print(vx)
    print(vy)
    print(math.cos(0.115846))