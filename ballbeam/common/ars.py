from time import time
from copy import deepcopy

import numpy as np
from scipy.special import expit
import gym
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import ListedColormap

# def param2vector(param):
#     vector = np.hstack([param[key] for key in param.keys()])
#     return vector
#
#
# def vector2param(vector, keys, sizes):
#     param = {}
#     start = 0
#     for key, size in zip(keys, sizes):
#         end = start + size
#         param[key] = vector[start:end]
#         start += size
#     return param

num_env_interacts = 0


class Controller:
    def __init__(self):
        self.myobs = np.zeros(2)

    # def policy(self, obs, param):
    #     # Linear observation policy
    #     return np.dot(param, obs)

    def policy(self, obs, param):
        # Linear observation policy w/ observation smoothing
        p = expit(param[2])
        self.myobs = (1-p)*obs + p*self.myobs
        return np.dot(param[0:2], self.myobs)

    def reset(self, obs=None):
        if obs is None:
            obs = np.zeros_like(self.myobs)
        self.myobs = np.copy(obs)
        return self.myobs


def rollout(param, length=200, pos0=None, render=False, verbose=False):
    global num_env_interacts

    done = True
    i = 0
    obs_hist = []
    rew_hist = []
    rew_tot = 0
    time_last = time()
    while i < length:
        if done:
            obs = env.reset(pos0)
            controller.reset(obs)  # only necessary for controllers with internal state/memory
            t = 0
        time_now = time()
        if render:
            env.render()
        action = controller.policy(obs, param)
        obs, rew, done, info = env.step(action)

        obs_hist.append(obs)
        rew_hist.append(rew)
        rew_tot += rew

        if verbose:
            spacer = '    '
            print(t+1, end='    ')
            print('%12.6f' % obs[0], end=spacer)
            print('%12.6f' % obs[1], end=spacer)
            print('%12.6f' % rew, end=spacer)
            print('%12.6f' % rew_tot, end=spacer)
            print('%12.6f' % (time_now - time_last), end=spacer)
            print('')

        time_last = time_now
        t += 1
        i += 1
        num_env_interacts += 1
    return rew_tot/length, rew_hist, obs_hist


def init_policy_search(num_inits=40, num_elites=4, length=200):
    """
    Initialize policy parameters
    num_inits:  Number of initial policies to try out
    num_elites:  Number of top performing initial policies to take weighted average of
    init_length:  Rollout length for initial policy evaluation
    """

    param_hist = []
    r_hist = []
    print('Initial policy search')
    for i in range(num_inits):
        # Randomize policy parameters
        param = np.array([env.np_random.uniform(low=0.0, high=4.0),
                          env.np_random.uniform(low=0.0, high=4.0),
                          env.np_random.uniform(low=-2.0, high=2.0)])
        r = rollout(param, length=length)[0]
        param_hist.append(deepcopy(param))
        r_hist.append(r)

        # Diagnostic
        spacer = '    '
        print(i + 1, end='  ')
        print('%12.6f' % r, end=spacer)
        print(param)
    # Choose top performers
    elite_idxs = np.argsort(-np.array(r_hist))[0:num_elites]
    elite_weights = np.ones(num_elites)
    # r_hist_elite = np.array([r_hist[idx] for idx in elite_idxs])
    # elite_weights = r_hist_elite - np.min(r_hist_elite)
    param = np.average([param_hist[idx] for idx in elite_idxs], axis=0, weights=elite_weights)
    print('')
    return param, param_hist, r_hist


def augmented_random_search(param0, num_epochs=10, num_deltas=2, num_elites=1, length=200, lr=1.0, delta_scale=0.2):
    """
    Augmented random search (ARS)
    param:  Initial policy parameters
    num_epochs:  Number of policy updates
    num_deltas:  Number of policy parameter search directions (per epoch)
    num_elites:  Number of top performing policy parameter search directions to keep (must be <= num_deltas)
    length:  Rollout length for policy search
    lr:  Learning rate (stepsize)
    delta_scale:  Policy parameter perturbation scale (std)
    """

    assert num_elites <= num_deltas
    param_hist = []
    r_hist = []
    param = deepcopy(param0)
    print('Augmented random search')
    for i in range(num_epochs):
        r_plu = []
        r_min = []
        deltas = []
        for j in range(num_deltas):
            # Perturb the policy parameters
            delta = env.np_random.normal(scale=delta_scale, size=param_shape)
            param_delta_plu = param + delta
            param_delta_min = param - delta
            deltas.append(delta)

            # Collect cumulative reward data
            rew_tot_plu, rew_hist_plu, obs_hist_plu = rollout(param_delta_plu, length=length)
            rew_tot_min, rew_hist_min, obs_hist_min = rollout(param_delta_min, length=length)
            r_plu.append(rew_tot_plu)
            r_min.append(rew_tot_min)

        # Choose top performers
        elite_idxs = np.argsort(-np.maximum(r_plu, r_min))[0:num_elites]

        # Reward stats
        rs = r_plu + r_min
        r_mean = np.mean(rs)
        r_mean_top = np.mean([[r_plu[idx]]+[r_min[idx]] for idx in elite_idxs])
        r_std = np.std(rs)

        r_best = np.max(rs)
        r_worst = np.min(rs)

        # Estimate gradient using weighted 2-point estimator w/ elitism
        g = np.zeros(param_shape)
        for k in elite_idxs:
            g += (r_plu[k]-r_min[k])*deltas[k]/num_elites

        # Update policy parameters
        param += (lr/r_std)*g

        param_hist.append(deepcopy(param))
        r_hist.append(r_mean)

        # Diagnostic
        spacer = '    '
        print(i + 1, end='  ')
        print('%12.6f' % r_mean, end=spacer)
        print('%12.6f' % r_mean_top, end=spacer)
        print('%12.6f' % r_best, end=spacer)
        print('%12.6f' % r_worst, end=spacer)
        print(param)
    print('')
    return param, param_hist, r_hist


def hist_plot(param, param0, param1, r, r0, r1, param_hist_inits, r_hist_inits, param_hist_ars, r_hist_ars):
    """
    Plot the policy parameter history as a contour/landscape
    """

    def policy_landscape(param_hist, r_hist, label=None, figax=None, cmap='viridis', show_scatter=False):
        XX = np.array([p[0] for p in param_hist])
        YY = np.array([p[1] for p in param_hist])
        ZZ = np.array(r_hist)

        fig, ax = plt.subplots() if figax is None else figax
        cf = ax.tricontourf(XX, YY, ZZ, levels=50, alpha=1.0, cmap=cmap)
        if show_scatter:
            ax.scatter(XX, YY, c='w', s=10, alpha=1.0, cmap=cmap, label='policy params')
        fig.colorbar(cf, label=label)
        return fig, ax

    fig, ax = plt.subplots(figsize=(10, 6))
    cmap1 = ListedColormap(cm.get_cmap('Blues', 512)(np.linspace(0.2, 0.6, 256)))
    cmap2 = ListedColormap(cm.get_cmap('Reds', 512)(np.linspace(0.3, 0.8, 256)))
    fig, ax = policy_landscape(param_hist_inits, r_hist_inits, figax=(fig, ax), cmap=cmap1, label='return (init)')
    fig, ax = policy_landscape(param_hist_ars, r_hist_ars, figax=(fig, ax), cmap=cmap2, label='return (ars)')
    ax.scatter(param0[0], param0[1], c='k', marker='o', label='Initial policy (ret=%.3f)'%r0)
    ax.scatter(param[0], param[1], c='k', marker='^', label='Trained policy (ret=%.3f)' % r)
    ax.scatter(param1[0], param1[1], c='k', marker='d', label='Handcrafted policy (ret=%.3f)' % r1)
    ax.legend()
    fig.tight_layout()


if __name__ == '__main__':
    controller = Controller()

    # Make the Gym environment and set the random seed
    env = gym.make('gym_ballbeam:BallBeam-v0', hardware=False)
    env.seed(None)

    # param_shape = env.observation_space.shape
    param_shape = (3,)

    param0, param_hist_inits, r_hist_inits = init_policy_search(num_inits=100,
                                                                num_elites=4,
                                                                length=200)
    param, param_hist_ars, r_hist_ars = augmented_random_search(param0,
                                                                num_epochs=50,
                                                                num_deltas=4,
                                                                num_elites=2,
                                                                length=200,
                                                                lr=0.5,
                                                                delta_scale=0.2)

    # Hand-crafted policy
    param1 = np.array([1.0, 0.5, 1.0])

    # Evaluate policies
    eval_length = 5*200

    print('evaluating initial policy...')
    r0 = rollout(param0, length=eval_length)[0]
    rollout(param0, length=200, pos0=0.100, render=True)

    print('evaluating learned policy...')
    r = rollout(param, length=eval_length)[0]
    rollout(param, length=200, pos0=0.100, render=True)

    print('evaluating handcrafted policy...')
    r1 = rollout(param1, length=eval_length)[0]
    rollout(param1, length=200, pos0=0.100, render=True)

    # Close the Gym environment
    env.close()

    # # Plot the policy parameter history
    # plt.close('all')
    # hist_plot(param, param0, param1, r, r0, r1, param_hist_inits, r_hist_inits, param_hist_ars, r_hist_ars)

    # Policy search on physical system starting from simulation learned policy
    env = gym.make('gym_ballbeam:BallBeam-v0', hardware=True)
    param, param_hist_ars, r_hist_ars = augmented_random_search(param,
                                                                num_epochs=20,
                                                                num_deltas=1,
                                                                num_elites=1,
                                                                length=200,
                                                                lr=0.5,
                                                                delta_scale=0.2)
    # Hand-crafted policy for hardware
    param3 = np.array([0.6, 0.4, 1.0])

    # Evaluate learned policy on the physical system
    print('evaluating learned policy on hardware...')
    rollout(param, length=200, verbose=True)[0]

    print('evaluating handcrafted policy on hardware...')
    rollout(param3, length=200, verbose=True)[0]

    env.close()


