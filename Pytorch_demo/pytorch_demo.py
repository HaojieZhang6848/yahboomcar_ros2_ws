#运行环境：python-3.6+torch-1.8.0+torchvision-0.9.0
#数据集：CIFAR-10，32*32大小的训练图50000张，测试图100000张
#导入相关的模块
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
import torch.optim as optim
#用均值和标准差归一化张量图像
transform = transforms.Compose(
    [transforms.ToTensor(),
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

#加载训练集数据，参数download为true时，表示如果该训练集不存在就先下载
cifar_train_data = torchvision.datasets.CIFAR10(root='./data', train=True,
                                          download=False, transform=transform)
#加载测试集数据
cifar_test_data = torchvision.datasets.CIFAR10(root='./data', train=False,
                                          transform=transform)
#打印训练集数据信息
print(cifar_train_data)
#打印训练集数据信息
print(cifar_test_data)
#统计训练集和测试集的数量
print(len(cifar_train_data.data)) 
print(len(cifar_test_data.data))

#将训练集和测试集的数据读取接口的输入按照batch size封装成Tensor
train_data_loader = torch.utils.data.DataLoader(cifar_train_data, batch_size=32, shuffle=True)
test_data_loader = torch.utils.data.DataLoader(cifar_test_data, batch_size=32, shuffle=True)

#定义一个卷积神经网络LeNet(2个卷积层conv1/conv2，3个全连接层fc1/fc2/fc3)
class LeNet(nn.Module):
    #定义网络需要的操作算子，比如卷积、全连接算子等等
    def __init__(self):
        super(LeNet, self).__init__()
		#Conv2d参数含义：输入channel数量，输出channel数量，内核大小
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(16*5*5, 120)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 10)
        self.pool = nn.MaxPool2d(2, 2)
    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(-1, 16*5*5)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
#把构建的网络数据复制到gpu上
cuda_device = torch.device("cuda:0")
net = LeNet().to(cuda_device)

#定义损失函数criterion和优化方法optimizer（使用SGD）
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(net.parameters(), lr=0.005, momentum=0.9)

print("开始训练...")
'''
训练过程包括：计算前向传播的输出，计算loss，反向梯度传播loss，优化
'''
for epoch in range(2):

	#记录100个batch的平均loss
    loss_in_100 = 0.0
    for i, data in enumerate(train_data_loader):
        inputs, labels = data
		 # 把数据复制到GPU
        inputs, labels = inputs.to(cuda_device), labels.to(cuda_device) # 注意需要复制到GPU
		#清零梯度，不清零的话会累计计算梯度
        optimizer.zero_grad()
		#利用创建的卷积网络计算输出
        outputs = net(inputs)
		#根据计算出的输出，计算出loss
        loss = criterion(outputs, labels)
		#反向梯度传播
        loss.backward()
		#优化
        optimizer.step()
        loss_in_100 += loss.item()
        if i % 100 == 99:
            print('[Epoch %d, Batch %5d] loss: %.5f' %
                  (epoch + 1, i + 1, loss_in_100 / 100))
            loss_in_100 = 0.0

print("训练完成！")

dataiter = iter(test_data_loader)
# 预测正确的数量和总数量
predict_correct = 0
total = 0

with torch.no_grad():
    print("开始测试...")	
    for data in test_data_loader:
        images, labels = data
		#把数据复制到gpu上，在gpu上做运算
        images, labels = images.to(cuda_device), labels.to(cuda_device)
        outputs = net(images)
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        predict_correct += (predicted == labels).sum().item()

print('10000张测试图的准确率为: %d %%' % (
    100 * predict_correct / total))























