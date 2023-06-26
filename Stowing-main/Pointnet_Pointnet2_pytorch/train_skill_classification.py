"""
Author: Benny
Date: Nov 2019
"""

import os
import pathlib
import sys
import torch
import numpy as np

import datetime
import logging
import provider
import importlib
import shutil
import argparse

from pathlib import Path
from tqdm import tqdm
from data_utils.SkillDataLoader import SkillClassificationDataset
from data_utils.ModelNetDataLoader import ModelNetDataLoader
from torch.utils.data import DataLoader
from config.config import gen_args
from utils.data_utils import batch_denormalize, batch_normalize
from utils.visualize import *

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
sys.path.append(os.path.join(ROOT_DIR, 'models'))

def parse_args():
    '''PARAMETERS'''
    parser = argparse.ArgumentParser('training')
    parser.add_argument('--use_cpu', action='store_true', help='use cpu mode')
    parser.add_argument('--gpu', type=str, default='0', help='specify gpu device')
    parser.add_argument('--batch_size', type=int, default=24, help='batch size in training')
    parser.add_argument('--model', default='pointnet2_skill', help='model name [default: pointnet_cls]')
    parser.add_argument('--num_category', default=3, type=int,  help='training on ModelNet10/40')
    parser.add_argument('--epoch', default=200, type=int, help='number of epoch in training')
    parser.add_argument('--learning_rate', default=0.00001, type=float, help='learning rate in training')
    parser.add_argument('--num_point', type=int, default=1024, help='Point Number')
    parser.add_argument('--optimizer', type=str, default='Adam', help='optimizer for training')
    parser.add_argument('--log_dir', type=str, default='pointnet2_skill', help='experiment root')
    parser.add_argument('--decay_rate', type=float, default=1e-4, help='decay rate')
    parser.add_argument('--use_normals', action='store_true', default=False, help='use normals')
    parser.add_argument('--process_data', action='store_true', default=False, help='save data offline')
    parser.add_argument('--use_uniform_sample', action='store_true', default=False, help='use uniform sampiling')
    parser.add_argument('--point_norm', action='store_true', default=True, help='normalize point cloud')

    return parser.parse_args()


def inplace_relu(m):
    classname = m.__class__.__name__
    if classname.find('ReLU') != -1:
        m.inplace=True


def test(args, model, loader, num_class, criterion):
    mean_correct = []
    class_acc = np.zeros((num_class, 3))
    classifier = model.eval()
    
    if args.point_norm:
        mean_p = torch.tensor(args.mean_p, device=args.device, dtype=torch.float32)
        std_p = torch.tensor(args.std_p, device=args.device, dtype=torch.float32)
    batch_loss = []

    for j, (points, target) in tqdm(enumerate(loader), total=len(loader)):
        if args.point_norm:
            points = batch_normalize(points.unsqueeze(1), mean_p, std_p).squeeze()
        points = points.transpose(2, 1)
        if not args.use_cpu:
            points = points.cuda()
            
        points_processed = [points[..., :args.n_particles+args.floor_dim], points[..., args.n_particles+args.floor_dim:]]
        
        if not args.use_cpu:
            target = target.cuda()

        pred, _ = classifier(*points_processed)
        loss = criterion(pred, target.long())
        batch_loss.append(loss.item())

        pred_choice = pred.data.max(1)[1]

        for cat in np.unique(target.cpu()):
            classacc = pred_choice[target == cat].eq(target[target == cat].long().data).cpu().sum()
            class_acc[cat, 0] += classacc.item() / float(points[target == cat].size()[0])
            class_acc[cat, 1] += 1

        correct = pred_choice.eq(target.long().data).cpu().sum()
        mean_correct.append(correct.item() / float(points.size()[0]))

    class_acc[:, 2] = class_acc[:, 0] / class_acc[:, 1]
    class_acc = np.mean(class_acc[:, 2])
    instance_acc = np.mean(mean_correct)

    return instance_acc, class_acc, np.mean(batch_loss)


def main(args):
    def log_string(str):
        logger.info(str)
        print(str)

    '''HYPER PARAMETER'''
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu

    '''CREATE DIR'''
    timestr = str(datetime.now().strftime('%Y-%m-%d_%H-%M'))
    exp_dir = Path(os.path.join(pathlib.Path(__file__).parent.resolve(),'log/') )
    exp_dir.mkdir(exist_ok=True)
    exp_dir = exp_dir.joinpath('classification')
    exp_dir.mkdir(exist_ok=True)
    backbone_dir = exp_dir.joinpath('pointnet2_ssg_wo_normals')

    if args.log_dir is None:
        exp_dir = exp_dir.joinpath(timestr)
    else:
        exp_dir = exp_dir.joinpath(args.log_dir)
        
    exp_dir.mkdir(exist_ok=True)
    checkpoints_dir = exp_dir.joinpath('checkpoints/')
    checkpoints_dir.mkdir(exist_ok=True)
    log_dir = exp_dir.joinpath('logs/')
    log_dir.mkdir(exist_ok=True)

    '''LOG'''
    config_args = gen_args()
    new_args = parse_args()
    # Update config_args with new_args
    config_args_dict = vars(config_args)
    config_args_dict.update(vars(new_args))
    args = argparse.Namespace(**config_args_dict)
    
    logger = logging.getLogger("Model")
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler = logging.FileHandler('%s/%s.txt' % (log_dir, args.model))
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    log_string('PARAMETER ...')
    log_string(args)

    '''DATA LOADING'''
    log_string('Load dataset ...')
    data_path = os.path.join(pathlib.Path(__file__).parent.resolve(), 'data/modelnet40_normal_resampled/')  

    if args.valid:
        phases = ['train', 'valid']
    else:
        phases = ['train']


    datasets = {phase: SkillClassificationDataset(args, phase) for phase in phases}
    dataloaders = {phase: DataLoader(datasets[phase], batch_size=args.batch_size, shuffle=True if phase == 'train' else False,
        num_workers=args.num_workers) for phase in phases}

    # train_dataset = ModelNetDataLoader(root=data_path, args=args, split='train', process_data=args.process_data)
    # test_dataset = ModelNetDataLoader(root=data_path, args=args, split='test', process_data=args.process_data)
    # trainDataLoader = torch.utils.data.DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=10, drop_last=True)
    # testDataLoader = torch.utils.data.DataLoader(test_dataset, batch_size=args.batch_size, shuffle=False, num_workers=10)

    '''MODEL LOADING'''
    num_class = args.num_category
    model = importlib.import_module(args.model)
    
    shutil.copy(os.path.join(pathlib.Path(__file__).parent.resolve(), 'models/%s.py' % args.model), str(exp_dir))
    shutil.copy(os.path.join(pathlib.Path(__file__).parent.resolve(), 'models/pointnet2_utils.py'), str(exp_dir))
    shutil.copy(os.path.join(pathlib.Path(__file__).parent.resolve(), 'train_classification.py'), str(exp_dir))

    classifier = model.get_model(num_class, normal_channel=args.use_normals, pretrained_path=str(backbone_dir) + '/checkpoints/best_model.pth')
    criterion = model.get_loss()
    classifier.apply(inplace_relu)

    if not args.use_cpu:
        classifier = classifier.cuda()
        criterion = criterion.cuda()

    try:
        checkpoint = torch.load(str(exp_dir) + '/checkpoints/best_model.pth')
        start_epoch = checkpoint['epoch']
        classifier.load_state_dict(checkpoint['model_state_dict'])
        log_string('Use pretrain model')
    except:
        log_string('No existing model, starting training from scratch...')
        start_epoch = 0

    if args.optimizer == 'Adam':
        optimizer = torch.optim.Adam(
            classifier.parameters(),
            lr=args.learning_rate,
            betas=(0.9, 0.999),
            eps=1e-08,
            weight_decay=args.decay_rate
        )
    else:
        optimizer = torch.optim.SGD(classifier.parameters(), lr=0.01, momentum=0.9)

    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.7)
    global_epoch = 0
    global_step = 0
    best_instance_acc = 0.0
    best_class_acc = 0.0
    
    if args.point_norm:
        mean_p = torch.tensor(args.mean_p, device=args.device, dtype=torch.float32)
        std_p = torch.tensor(args.std_p, device=args.device, dtype=torch.float32)

    '''TRANING'''
    logger.info('Start training...')
    train_losses = []
    valid_losses = []
    for epoch in range(start_epoch, args.epoch):
        log_string('Epoch %d (%d/%s):' % (global_epoch + 1, epoch + 1, args.epoch))
        mean_correct = []
        classifier = classifier.train()

        scheduler.step()
        batch_loss = []
        for batch_id, (points, target) in tqdm(enumerate(dataloaders['train'], 0), total=len(dataloaders['train']), smoothing=0.9):
            optimizer.zero_grad()
            
            if args.point_norm:
                points = batch_normalize(points.unsqueeze(1), mean_p, std_p).squeeze()
            # points = points.cpu().data.numpy()
            # points = provider.random_point_dropout(points)
            # points[:, :, 0:3] = provider.random_scale_point_cloud(points[:, :, 0:3])
            # points[:, :, 0:3] = provider.shift_point_cloud(points[:, :, 0:3])
            # points = torch.Tensor(points)
            points = points.transpose(2, 1)
            if not args.use_cpu:
                points = points.cuda()
                
            points_processed = [points[..., :args.n_particles+args.floor_dim], points[..., args.n_particles+args.floor_dim:]]


            if not args.use_cpu:
                target = target.cuda()

            pred, _  = classifier(*points_processed)
            loss = criterion(pred, target.long())
            batch_loss.append(loss.item())

            pred_choice = pred.data.max(1)[1]

            correct = pred_choice.eq(target.long().data).cpu().sum()
            mean_correct.append(correct.item() / float(points.size()[0]))
            loss.backward()
            optimizer.step()
            global_step += 1
        train_loss = np.mean(batch_loss)

        log_string(f'Train Loss: {train_loss}')

        train_losses.append(train_loss)

        train_instance_acc = np.mean(mean_correct)
        log_string('Train Instance Accuracy: %f' % train_instance_acc)

        with torch.no_grad():
            instance_acc, class_acc, valid_loss = test(args, classifier.eval(), dataloaders['valid'], num_class=num_class, criterion=criterion)
            valid_losses.append(valid_loss)
            log_string(f'Valid Loss: {valid_loss}')

            if (instance_acc >= best_instance_acc):
                best_instance_acc = instance_acc
                best_epoch = epoch + 1

            if (class_acc >= best_class_acc):
                best_class_acc = class_acc
            log_string('Test Instance Accuracy: %f, Class Accuracy: %f' % (instance_acc, class_acc))
            log_string('Best Instance Accuracy: %f, Class Accuracy: %f' % (best_instance_acc, best_class_acc))

            if (instance_acc >= best_instance_acc):
                logger.info('Save model...')
                savepath = str(checkpoints_dir) + '/best_model.pth'
                log_string('Saving at %s' % savepath)
                state = {
                    'epoch': best_epoch,
                    'instance_acc': instance_acc,
                    'class_acc': class_acc,
                    'model_state_dict': classifier.state_dict(),
                    'optimizer_state_dict': optimizer.state_dict(),
                }
                torch.save(state, savepath)
            global_epoch += 1



    logger.info('End of training...')


if __name__ == '__main__':
    args = parse_args()
    main(args)
