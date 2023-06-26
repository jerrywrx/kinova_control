import torch.nn as nn
import torch.nn.functional as F
from pointnet2_utils import PointNetSetAbstraction
import torch
from pointnet2_cls_ssg import get_model as get_cls_model  # assuming pointnet2_cls_ssg.py is in the same directory

class get_model(nn.Module):
    def __init__(self, num_class, normal_channel=False, pretrained_path="path_to_pretrained_weights.pth"):
        super(get_model, self).__init__()

        # Initialize pretrained model and load weights
        self.pretrained_model = get_cls_model(num_class=40, normal_channel=False)
        checkpoint = torch.load(pretrained_path)
        self.pretrained_model.load_state_dict(checkpoint['model_state_dict'])


        # Freeze pretrained model parameters
        for param in self.pretrained_model.parameters():
            param.requires_grad = False

        # MLP to process concatenated point cloud features
        self.fc_concat1 = nn.Linear(2048, 256)  # assuming that the output of concatenated features is 1024 (512 from each PointNet)
        self.bn_concat1 = nn.BatchNorm1d(256)
        self.fc_concat2 = nn.Linear(256, 64)
        self.bn_concat2 = nn.BatchNorm1d(64)
        self.fc_concat3 = nn.Linear(64, num_class)

    def forward(self, xyz_in, xyz_out):
        # compute features for input point cloud
        _, l3_points_in = self.pretrained_model(xyz_in)

        # compute features for output point cloud
        _, l3_points_out = self.pretrained_model(xyz_out)

        # concatenate point cloud features
        x_in = l3_points_in.view(xyz_in.shape[0], 1024)
        x_out = l3_points_out.view(xyz_out.shape[0], 1024)
        x_feature = torch.cat((x_in, x_out), dim=1)  # concatenate along feature dimension

        # Pass through MLP
        x = F.relu(self.bn_concat1(self.fc_concat1(x_feature)))
        x = F.relu(self.bn_concat2(self.fc_concat2(x)))
        x = self.fc_concat3(x)  # removed log_softmax

        return x, x_feature
    
class get_loss(nn.Module):
    def __init__(self):
        super(get_loss, self).__init__()

    def forward(self, pred, target):
        total_loss = F.cross_entropy(pred, target)

        return total_loss

# class get_model(nn.Module):
#     def __init__(self,num_class,normal_channel=False):
#         super(get_model, self).__init__()
#         in_channel = 6 if normal_channel else 3
#         self.normal_channel = normal_channel
#         self.sa1 = PointNetSetAbstraction(npoint=512, radius=0.2, nsample=32, in_channel=in_channel, mlp=[64, 64, 128], group_all=False)
#         self.sa2 = PointNetSetAbstraction(npoint=128, radius=0.4, nsample=64, in_channel=128 + 3, mlp=[128, 128, 256], group_all=False)
#         self.sa3 = PointNetSetAbstraction(npoint=None, radius=None, nsample=None, in_channel=256 + 3, mlp=[256, 512, 1024], group_all=True)
#         self.fc1 = nn.Linear(1024, 512)
#         self.bn1 = nn.BatchNorm1d(512)
#         self.drop1 = nn.Dropout(0.4)
#         self.fc2 = nn.Linear(512, 256)
#         self.bn2 = nn.BatchNorm1d(256)
#         self.drop2 = nn.Dropout(0.4)
#         self.fc3 = nn.Linear(256, 128)
        
#         # MLP to process concatenated point cloud features
#         self.fc_concat1 = nn.Linear(256, 128)  # assuming that the output of concatenated features is 512 (256 from each PointNet)
#         self.bn_concat1 = nn.BatchNorm1d(128)
#         self.fc_concat2 = nn.Linear(128, 64) 
#         self.bn_concat2 = nn.BatchNorm1d(64)
#         self.fc_concat3 = nn.Linear(64, num_class) 

#     def forward(self, xyz_in, xyz_out):
#         B, _, _ = xyz_in.shape
#         if self.normal_channel:
#             norm_in = xyz_in[:, 3:, :]
#             xyz_in = xyz_in[:, :3, :]
#             norm_out = xyz_out[:, 3:, :]
#             xyz_out = xyz_out[:, :3, :]
#         else:
#             norm_in = None
#             norm_out = None
        
#         # compute features for input point cloud
#         l1_xyz_in, l1_points_in = self.sa1(xyz_in, norm_in)
#         l2_xyz_in, l2_points_in = self.sa2(l1_xyz_in, l1_points_in)
#         l3_xyz_in, l3_points_in = self.sa3(l2_xyz_in, l2_points_in)
        
#         # compute features for output point cloud
#         l1_xyz_out, l1_points_out = self.sa1(xyz_out, norm_out)
#         l2_xyz_out, l2_points_out = self.sa2(l1_xyz_out, l1_points_out)
#         l3_xyz_out, l3_points_out = self.sa3(l2_xyz_out, l2_points_out)

#         # concatenate point cloud features
#         x_in = l3_points_in.view(B, 1024)
#         x_out = l3_points_out.view(B, 1024)

#         x_in = self.drop1(F.relu(self.bn1(self.fc1(x_in))))
#         x_in = self.drop2(F.relu(self.bn2(self.fc2(x_in))))
#         x_in = self.fc3(x_in)
#         x_out = self.drop1(F.relu(self.bn1(self.fc1(x_out))))
#         x_out = self.drop2(F.relu(self.bn2(self.fc2(x_out))))
#         x_out = self.fc3(x_out)

#         x = torch.cat((x_in, x_out), dim=1)  # concatenate along feature dimension

#         # Pass through MLP
#         x = F.relu(self.bn_concat1(self.fc_concat1(x)))
#         x = F.relu(self.bn_concat2(self.fc_concat2(x)))
#         x = F.log_softmax(self.fc_concat3(x), dim=-1)  # use fc_concat instead of fc3

#         return x



