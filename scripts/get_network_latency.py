from pythonping import ping
import matplotlib.pyplot as plt
import argparse
import seaborn as sns
import pandas as pd
import os

def ping_host(host):
    ping_result = ping(target=host, count=1000, timeout=2)

    return {
        'host': host,
        'avg_latency': ping_result.rtt_avg_ms,
        'min_latency': ping_result.rtt_min_ms,
        'max_latency': ping_result.rtt_max_ms,
        'packet_loss': ping_result.packet_loss
    }


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--exp', type=str, default='ping_latency_check', help='Experiment description')
    parser.add_argument('--savedir', type=str, default='/home/robotlab/projects/bimanual-infra/scripts/network-latency-plots', help='Directory to save experiments')
    
    args = parser.parse_args()

    # create directory to save experiment
    os.makedirs("{}/{}".format(args.savedir, args.exp), exist_ok=True)

    hosts = [
        (0, '192.168.86.230'),
        (9, '192.168.86.216')
    ]

    palette = sns.color_palette("Spectral", n_colors=10)


    for color, host in hosts:

        host_latencies = {
            'try': [],
            'max_latencies' : [],
            'min_latencies' : [],
            'avg_latencies' : []
        }
        
        for i in range(10):
            print("Host : {} Iteration: {}".format(host, i))
            print(ping_host(host))
            host_latencies['try'].append(i)
            host_latencies['max_latencies'].append(ping_host(host)['max_latency'])
            host_latencies['min_latencies'].append(ping_host(host)['min_latency'])
            host_latencies['avg_latencies'].append(ping_host(host)['avg_latency'])
        
        host_latencies_df = pd.DataFrame(host_latencies)
        sns.lineplot(data = host_latencies_df, x='try', y='avg_latencies', label='avg_latency_{}'.format(host), color=palette[color], linewidth=2)
        plt.fill_between(host_latencies['try'], host_latencies['min_latencies'], host_latencies['max_latencies'], color=palette[color], alpha=0.1)

        # save dataframes to csv
        host_latencies_df.to_csv("{}/{}/{}_latencies.csv".format(args.savedir, args.exp, host))

        # # plot the times
        # plt.plot(host_latencies['max_latencies'], label="Max Latency - {}".format(host))
        # plt.plot(host_latencies['min_latencies'], label="Min Latency - {}".format(host))
        # plt.plot(host_latencies['avg_latencies'], label="Avg Latency - {}".format(host))

    # Put all plots on the same graph
    plt.title("Network Latency for Hosts: {}".format(hosts))
    plt.xlabel("Try")
    plt.ylabel("Latency (ms)")
    plt.legend()
    plt.savefig("{}/{}/latencies.png".format(args.savedir, args.exp))
