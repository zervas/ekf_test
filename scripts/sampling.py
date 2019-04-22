import math
import numpy as np
import scipy.stats
import timeit
import matplotlib.pyplot as plt


def sample_normal_twelve(mu, sigma):
    """ Sample from a normal distribution using 12 uniform samples;
    """

    # Return sample from N(0, sigma)
    x = 0.5 * np.sum(np.random.uniform(-sigma, sigma, 12))
    return mu + x

def sample_normal_rejection(mu, sigma):
    """Sample from a normal distribution using rejection sampling.
    See lecture on probabilistic motion models slide 25 for details.
    """
    # Length of interval from wich samples are drawn
    interval = 5*sigma
    # Maximum value of the pdf of the desired normal distribution
    max_density = scipy.stats.norm(mu,sigma).pdf(mu)
    # Rejection loop
    while True:
        x = np.random.uniform(mu - interval, mu + interval, 1)[0]
        y = np.random.uniform(0, max_density, 1)
        if y <= scipy.stats.norm(mu, sigma).pdf(x):
            break
    return x

def sample_normal_boxmuller(mu, sigma):
    """Sample from a normal distribution using Box-Muller method.
    See exercise sheet on sampling and motion models.
    """
    # Two uniform random variables
    u = np.random.uniform(0, 1, 2)
    # Box-Muller formula returns sample from STANDARD normal distribution
    x = math.cos(2*np.pi*u[0])

def evaluate_sampling(mu, sigma, n_samples):
    n_bins = 100
    samples = []
    for i in range (n_samples):
        samples.append(sample_normal_twelve(mu, sigma))
    plt.figure()
    count, bins, ignored = plt.hist(samples, n_bins, density=True)
    plt.plot(bins, scipy.stats.norm(mu, sigma).pdf(bins), linewidth=2, color='r')
    plt.xlim([mu - 5*sigma, mu + 5*sigma])
    plt.title("12 samples")

def main():
    sample = sample_normal_twelve(0, 1)
    print(sample)
    evaluate_sampling(0, 1, 1000)

    plt.show()


if __name__ == '__main__':
    main()