import numpy as np
from scipy import fftpack
import matplotlib.pyplot as plt


def plot_fast_fourier_transform(sig, dt, N):
    FFT_freq = fftpack.fftfreq(len(sig), dt)
    sig_FFT = fftpack.fft(sig)
    plt.plot(FFT_freq[:N//2], np.abs(sig_FFT[:N//2]))  # Half the signal to get only positive frequencies
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("FFT Amplitude")
    plt.grid()


def plot_lowpass_filtered_signal(sig, t, dt):
    """
    Removes random noise from input signal

    """
    FFT_freq = fftpack.fftfreq(len(sig), dt)
    sig_FFT = fftpack.fft(sig)
    amplitude = np.abs(sig_FFT)  # FFT signal amplitudes (and complex to real conversion)
    amp_position = amplitude.argmax()  # Index of the max element of amplitude
    peak_freq = FFT_freq[amp_position]
    sig_FFT[np.abs(FFT_freq) > peak_freq] = 0  # Filters out frequencies above FFT peak frequency
    filtered_sig = fftpack.ifft(sig_FFT)  # Inverse of FFT
    plt.plot(t, sig, color='r', label='input signal')
    plt.plot(t, filtered_sig, color='b', label='filtered signal')
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid()


def plot_signal(sig, t):
    plt.plot(t, sig)
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.grid()


def generate_sinusoidal_signal(T, N):
    t = np.linspace(0, T, N)
    dt = t[1] - t[0]  # time step per sample
    # dt = np.diff(t)[0]
    f1 = 20/(N*dt)
    f2 = 10/(N*dt)
    f3 = (10 + 5*N)/(N*dt)
    print(f"f1 = {f1}")
    print(f"f2 = {f2}")
    print(f"f3 = {f3}")
    # signal = 1.7*np.sin(2*np.pi*f1*t) + 0.5*np.sin(2*np.pi*f2*t) + 0.3*np.random.randn(len(t))
    signal = 2*np.sin(2*np.pi*f2*t) + 0.5*np.random.randn(len(t))
    # signal = np.sin(2*np.pi*f3*t) + 0.1*np.random.randn(len(t))
    return signal, t, dt


# %% Main program
if __name__ == '__main__':
    T = 30  # Seconds to measure signal
    N = 200  # Amount of samples for FFT
    sinusoidal_signal, time, time_step = generate_sinusoidal_signal(T, N)

    plt.figure("Figure 1")
    plot_signal(sinusoidal_signal, time)
    plt.figure("Figure 2")
    plot_fast_fourier_transform(sinusoidal_signal, time_step, N)
    plt.figure("Figure 3")
    plot_lowpass_filtered_signal(sinusoidal_signal, time, time_step)
    plt.show()


# from scipy.fft import fft, ifft
# from scipy.fft import fft, fftfreq


# def main():
    # time_step = 0.05
    # time_vec = np.arange(0, 10, time_step)
    # sig = 3*np.sin(2*np.pi*time_vec) + np.random.randn(time_vec.size)
    # sig_fft = fftpack.fft(sig)
    # amplitude = np.abs(sig_fft)
    # power = amplitude**2
    # angle = np.angle(sig_fft)
    # sample_freq = fftpack.fftfreq(sig.size, d=time_step)
    # amp_freq = np.array([amplitude, sample_freq])
    # amp_position = amp_freq[0, :].argmax()
    # peak_freq = amp_freq[1, amp_position]
    # high_freq_fft = sig_fft.copy()
    # high_freq_fft[np.abs(sample_freq) > peak_freq] = 0
    # filtered_sig = fftpack.ifft(high_freq_fft)
    # plt.semilogy(time_vec, filtered_sig, '-b')
    # plt.semilogy(time_vec, sig, '-r')
    # plt.semilogy(sample_freq, amplitude, '-g')
    # plt.legend(['filtered signal', 'input signal', 'FFT'])
    # plt.grid()
    # plt.show()

    # N = 600
    # T = 1.0 / 800.0
    # x = np.linspace(0.0, N * T, N, endpoint=False)
    # y = np.sin(50.0 * 2.0 * np.pi * x) + 0.5 * np.sin(80.0 * 2.0 * np.pi * x)
    # yf = fft(y)
    # xf = fftfreq(N, T)[:N // 2]
    # plt.plot(xf, 2.0 / N * np.abs(yf[0:N // 2]))
    # plt.grid()
    # plt.show()

    # x = np.array([1.0, 2.0, 1.0, -1.0, 1.5])
    # y = fft(x)
    # print(y)
    # yinv = ifft(y)
    # print(yinv)
    # print(np.sum(x))
