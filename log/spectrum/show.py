import matplotlib.pyplot as plt

def read(file_name):
    result = []
    with open(file_name) as f:
        data = f.readlines()
        for d in data:
            result.append(float(d))

    return result

def test1():
    import math
    withoutbias = read("/home/evan/code/radar-localization/log/spectrum/spectrum_withoutbias.txt")
    afterfilet = read("/home/evan/code/radar-localization/log/spectrum/spectrum_afterfilter.txt")
    plt.figure(1)
    size = len(withoutbias) + 1
    plt.plot(list(range(1, size)), withoutbias)
    plt.title("without bias")
    
    res = 0
    for i in list(range(1, size - 1)):
        if i > 2000:
            res = res + withoutbias[i] * withoutbias[i]
    res = math.sqrt(res / (size - 2000))
    print(res)


    plt.figure(2)
    size = len(afterfilet) + 1
    plt.plot(list(range(1, size)), afterfilet)
    plt.title("filtered data")
    
    plt.show()

def test2():
    result = read("/home/evan/code/radar-localization/log/my/spectrum_mean_power.txt")
    plt.figure(1)
    size = len(result) + 1
    plt.plot(list(range(1, size)), result)
    plt.title("mean power")
    plt.show()

if __name__ == "__main__":
    test1()