from scipy import interpolate
from scipy.signal import butter, filtfilt, find_peaks
import numpy as np
import padasip as pa


class MotionPrediction:
    def __init__(self):
        self.fs = None
        self.z_pred = None
        self.t0 = None

        self.is_ready = False

        self.period = None

        self.filt_param = None
        
        self.time_varying = None
        self.calc_coeff = None

        self.order = None
        self.coeff = None
        
        self.rls_filter = None
        self.mu = 0.001

    def initialize(self, t, z, order=2, fs=60., filt_order=12, filt_freq=5., time_varying=False, calc_coeff=False):
        self.fs = float(fs)
        self.time_varying = time_varying
        self.calc_coeff = calc_coeff
        self.order = order
        
        self.filt_param = butter(filt_order, filt_freq, btype='low', fs=fs)
        
        if calc_coeff:
            self.rls_filter = pa.filters.FilterRLS(n=self.order, mu=self.mu, w='zeros')
        else:
            self.coeff = np.ones((self.order,)) / float(self.order)
        
        if self.update(t, z):
            self.is_ready = True
            return True
        else:
            return False
        
    def update(self, t, z):
        if np.max(np.abs(z)) < 0.8E-3:
            self.reset()
            return False
        
        t_rs = np.arange(t[0], t[-1], 1. / self.fs)
        if t_rs[-1] > t[-1]:
            t_rs = t_rs[:-1]
        z_interp = interpolate.interp1d(t, z, kind='cubic')
        z_rs = z_interp(t_rs)

        z_flt = filtfilt(self.filt_param[0], self.filt_param[1], z_rs)

        mean_z = np.mean(z_flt)
        
        z_ac = np.correlate(z_flt - mean_z, z_flt - mean_z, 'full')
        z_ac = z_ac[z_flt.size+1:] + z_ac[:z_flt.size:-1]
            
        pks = find_peaks(z_ac, distance=2. * self.fs, height=0.25 * np.amax(z_ac))[0]

        if pks.size <= self.order:
            self.reset()
            return False
        
        period_sample = int(round(np.mean(pks[1:] - pks[0:-1])))
        
        self.t0 = t_rs[-1]

        self.z_pred = interpolate.interp1d(t_rs, z_flt, kind='cubic')
            
        if self.time_varying:
            cf = np.correlate(z_flt[-period_sample:] - mean_z, (z_flt - mean_z), 'full')
            
            pks = find_peaks(cf, distance=2. * self.fs, height=0.5 * np.amax(cf))[0]
            p = (pks[1:] - pks[:-1])
            self.period = p[0:self.order] / self.fs
            
            if self.calc_coeff:
                p_max = np.amax(p)
                
                a = np.vstack((z_flt[-p_max:], z_flt[-p_max-p[0]:-p[0]]))
                for i in range(2, self.order):
                    dp = np.sum(p[0:i])
                    a = np.vstack((a, z_flt[-p_max-dp:-dp]))
                d = z_flt[-p_max:]
                self.coeff = self.rls_filter.run(d, a.T)[2]
                self.coeff = self.coeff[-1, :]
        else:
            self.period = period_sample / self.fs
        
            if self.calc_coeff:
                a = np.vstack((z_flt[-period_sample:], z_flt[-2 * period_sample:-period_sample]))
                for i in range(2, self.order):
                    a = np.vstack((a, z_flt[-(i + 1) * period_sample:-i * period_sample]))
                d = z_flt[-period_sample:]
                self.coeff = self.rls_filter.run(d, a.T)[2]
                self.coeff = self.coeff[-1, :]
        return True

    def get_pred(self, t):
        z = 0.
        if self.time_varying:
            if t >= self.t0 + self.period[0]:
                n = (t - self.t0) // self.period[0]
                t = t - n * self.period[0]
            for i in range(0, self.order):
                dp = np.sum(self.period[0:i+1])
                z = z + self.coeff[i] * self.z_pred(t - dp)
        else:
            if t >= self.t0 + self.period:
                n = (t - self.t0) // self.period
                t = t - n * self.period
            for i in range(0, self.order):
                z = z + self.coeff[i] * self.z_pred(t - (i + 1) * self.period)
        return z

    def reset(self):
        self.is_ready = False
        self.z_pred = None
