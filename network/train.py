import tqdm
import utils
import torch
import numpy as np
from torch import nn
from torch import optim
from torchsummary import summary
from network import SpringNetwork
from dataset import SimulatorDataset

if __name__=='__main__':

    # Set some parameters
    device = torch.device('cuda')
    root = Path('checkpoints')
    train_path = '../dataset/train'
    val_path = '../dataset/val'
    lr = 0.0001
    momentum = 0.9
    batch_size = 8
    num_workers = 8
    n_epochs = 500
    validate_each = 5

    train_dataset = SimulatorDataset(train_kinematics_file=train_path+'_kinematics.csv', simulator_file=train_path+'_simulator.csv', label_file=train_path+'_polaris.csv')
    val_dataset = SimulatorDataset(val_kinematics_file=val_path+'_kinematics.csv', simulator_file=val_path+'_simulator.csv', label_file=val_path+'_polaris.csv')
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    val_loader = DataLoader(dataset=val_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
    
    # If loading from previous model
    use_previous_model = False
    epoch_to_use = 0

    net = SpringNetwork
    loss_fn = nn.MSELoss()
    optimizer = optim.SGD(net.parameters(), lr=lr, momentum=momentum)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer)

    # Make directories for saving models and results
    try:
        model_root = root / "models"
        model_root.mkdir(mode=0o777, parents=False)
    except OSError:
        print("path exists")

    try:
        results_root = root / "results"
        results_root.mkdir(mode=0o777, parents=False)
    except OSError:
        print("path exists")

    # Load previous model if requested
    if use_previous_model:
        path = model_root / 'model_{}.pt'.format(epoch_to_use)
        if path.exists():
            state = torch.load(str(path))
            epoch = state['epoch'] + 1
            step = state['step']
            net.load_state_dict(state['model'])
            optimizer.load_state_dict(state['optimizer'])
            scheduler.load_state_dict(state['scheduler'])

            print('Restored model, epoch {}, step {:,}'.format(epoch, step))
        else:
            print('Failed to restore model')
            exit()
    else:
        epoch = 1
        step = 0
        best_mean_error = 0.0
        net = utils.init_net(net)
        summary(net)

    # List things to save
    save = lambda ep, model, model_path, error, optimizer, scheduler: torch.save({
        'model': model.state_dict(),
        'epoch': ep,
        'step': step,
        'error': error,
        'optimizer': optimizer.state_dict(),
        'scheduler': scheduler.state_dict()
    }, str(model_path))


    try:    
        for e in range(epoch, n_epochs + 1):
            scheduler.step()
            for param_group in optimizer.param_groups:
                print('Learning rate ', param_group['lr'])
        
            net.train()

            tq = tqdm.tqdm(total=(len(train_loader) * batch_size))
            tq.set_description('Epoch {}, lr {}'.format(epoch, lr))
            epoch_loss = 0

            for i, (input_data, label_data) in enumerate(train_loader):
                input_data, label_data = input_data.to(device), label_data.to(device)
                pred  = net(input_data)
                loss = loss_fn(pred, label_data)
                epoch_loss += loss.item()

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                mean_loss = np.mean(loss.item())
                tq.update(batch_size)
                tq.set_postfix(loss=' loss={:.5f}'.format(mean_loss))
                step += 1
                
            if e % validate_each == 0:
                torch.cuda.empty_cache()
                counter=  0
                with torch.no_grad():
                    net.eval()
                    for j, (input_data, label_data) in enumerate(val_loader):
                        input_data, label_data = input_data.to(device), label_data.to(device)
                        pred = net(input_data)
                        val_loss = loss_fn(pred, label_data)
                        all_val_loss.append(loss.item())
                            
                mean_loss = np.mean(all_val_loss)
                tq.set_postfix(loss='validation loss={:5f}'.format(mean_loss))

                best_mean_rec_loss = mean_loss
                model_path = model_root / "model_{}.pt".format(epoch)
                save(epoch, net, model_path, best_mean_rec_loss, optimizer, scheduler)
            
            utils.write_event(log, step, loss=mean_loss)
            tq.close()

    except KeyboardInterrupt:
        tq.close()
        print('Ctrl+C, done.')
        exit()

