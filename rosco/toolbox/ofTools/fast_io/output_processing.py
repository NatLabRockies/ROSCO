''' 
A number of the file processing tools used here were provided by or modified from: https://github.com/ebranlard/weio. 

Functions:
--------
run_openfast
load_fast_out
load_ascii_output
load_binary_output
trim_output
'''
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms
from itertools import takewhile
import struct
import multiprocessing as mp

from rosco.toolbox.ofTools.util import spectral

class output_processing():
    '''
    Some plotting utilities for OpenFAST data. 

    Methods:
    load_fast_out
    plot_fast_out
    plot_spectral
    '''

    def __init__(self, filenames=[], cases=None, tmin=None, tmax=None, verbose=False):
        
        # Define generic plotting cases
        if cases:
            self.plot_cases=cases
        else:
            self.plot_cases = {'Baseline': ['Wind1VelX', 'GenPwr', 'RotSpeed', 'BldPitch1', 'GenTq']}

        if len(filenames) > 0:
            self.load_fast_out(filenames, tmin=tmin, tmax=tmax, verbose=verbose)

    def load_fast_out(self, filenames, tmin=None, tmax=None, verbose=False, max_cores=4):
        """Load a FAST binary or ascii output file
        
        Parameters
        ----------
        filenames : list
            list of filenames
        tmin : float, optional
            initial time to trim output data to 
        tmax : float, optional
            final data to trim output data to
        verbose : bool, optional
            Print updates
        max_cores: int, optional
            Maximum number of cores to use for loading outputs in parallel
        
        Returns
        -------
        fastout: list
            List of dictionaries containing OpenFAST output data.
        """
        if type(filenames) is str:
            filenames = [filenames]

        cores = min(len(filenames), max_cores)
            
        try:
            self.fastout
        except AttributeError:
            self.fastout = []
        
        inputs = []
        for filename in filenames:
            inp = {}
            inp['filename'] = filename
            inp['verbose'] = verbose
            inputs.append(inp)

        if cores > 1:
            pool = mp.Pool(cores)
            self.fastout = pool.map(self._load_fast_data, inputs)
            pool.close()
            pool.join()
        else:
            for inp in inputs:
                fast_data = self._load_fast_data(inp)
                self.fastout.append(fast_data)

        # Trim outputs
        if (tmin) or (tmax):
            trim_output(self.fastout, tmin=tmin, tmax=tmax, verbose=verbose)

        # return fastout
        return self.fastout
    
    def _load_fast_data(self,input):
        # Unpack
        filename = input['filename']
        verbose = input['verbose']

        assert os.path.isfile(filename), "File, %s, does not exists" % filename
        with open(filename, 'r') as f:
            if verbose:
                print('Loading data from {}'.format(filename))
            try:
                f.readline()
            except UnicodeDecodeError:
                data, info = load_binary_output(filename)
            else:
                data, info = load_ascii_output(filename)

        # Build dictionary
        fast_data = dict(zip(info['channels'],data.T))
        fast_data['meta'] = info
        fast_data['meta']['filename'] = filename
        return fast_data

    def plot_fast_out(self, fastout=None, cases=None, showplot=True, fignum=None, xlim=None):
        '''
        Plots OpenFAST outputs for desired channels

        Parameters:
        -----------
        cases : dict
            Dictionary of lists containing desired outputs
        fastout : list
            List of dictionaries of OpenFAST output information, output from load_fast_out
        showplot: bool, optional
            Show the plot
        fignum: int, optional
            Define figure number. Note: Should only be used when plotting a singular case. 

        Returns:
        --------
        figlist: list
            list of figure handles
        axeslist: list
            list of axes handles
        '''

        if not fastout:
            try:
                fastout = self.fastout
            except:
                raise AttributeError('Cannot plot OpenFAST output data before it is loaded with load_fast_out.')
        if not cases:
            cases = self.plot_cases

        figlist = []
        axeslist = []
        # Plot cases
        for case in cases.keys():
            # channels to plot
            channels = cases[case]
            # instantiate plot and legend
            fig, axes = plt.subplots(len(channels), 1, sharex=True, num=fignum, constrained_layout=True)
            myleg = []
            for fast_out in fastout:      # Multiple channels
                # write legend
                Time = fast_out['Time']
                myleg.append(fast_out['meta']['name'])
                if len(channels) > 1:  # Multiple channels
                    for axj, channel_tup in zip(axes, channels):
                        if not isinstance(channel_tup, tuple):
                            channel_tup = (channel_tup,)
                        for cidx, channel in enumerate(channel_tup):
                            try:
                                # plot
                                axj.plot(Time, fast_out[channel])
                                # label
                                unit_idx = fast_out['meta']['channels'].index(channel)
                            except:
                                print('{} is not available as an output channel.'.format(channel))
                        axj.set_ylabel('{:^} \n ({:^})'.format(
                            '\n'.join(channel_tup),
                            fast_out['meta']['attribute_units'][unit_idx]))
                        axj.grid(True)
                    axes[0].set_title(case)
                    
                else:                       # Single channel
                    if not isinstance(channels[0], tuple):
                        channel_tup = (channels[0],)
                    else:
                        channel_tup = channels[0]
                    for cidx, channel in enumerate(channel_tup):
                        try:
                            # plot
                            axes.plot(Time, fast_out[channel])
                            # label
                            channel.replace(' ','\n')
                            axes.grid(True)
                            axes.set_title(case)
                        except:
                            print('{} is not available as an output channel.'.format(channel))
                        axes.set_ylabel('{:^} \n ({:^})'.format(
                            '\n'.join(channel_tup),
                            fast_out['meta']['attribute_units'][unit_idx]))
                
                plt.legend(myleg, loc='upper center', bbox_to_anchor=(
                    0.5, 0.0), borderaxespad=2, ncol=len(fastout))

            
            fig.align_ylabels()
            figlist.append(fig)
            axeslist.append(axes)

            if xlim:
                plt.xlim(xlim)

        if showplot:
            plt.show()

        return figlist, axeslist

    def plot_spectral(self, fastout=None, cases=None,
                      averaging='None', averaging_window='Hann', detrend=False, nExp=None,
                      show_RtSpeed=False, RtSpeed_idx=None,
                      add_freqs=None, add_freq_labels=None,
                      showplot=False, fignum=None):
        '''
        Plots OpenFAST outputs for desired channels

        Parameters:
        -----------
        fastout : dict
            Dictionary of OpenFAST output information, output from load_fast_out
        cases : list of tuples (str, int)
            Dictionary of lists containing desired outputs. 
            Of the format (channel, case), i.e. [('RotSpeed', 0)]
        averaging: str, optional
            PSD averaging method. None, Welch
        averaging_window: str, optional
            PSD averaging window method. Hamming, Hann, Rectangular
        detrend: bool, optional
            Detrend data?
        nExp: float, optional
            Exponent for hamming windowing
        show_RtSpeed: Bool, optional
            Plot 1p and 3p rotor speeds for simulation cases plotted
        RtSpeed_idx: ind, optional
            Specify the index for the simulation case that the rotor speed is plotted from. 
        add_freqs: list, optional
            List of floats containing additional frequencies to plot lines of
        add_freq_labels: list, optional
            List of strings to label add_freqs
        showplot: bool, optional
            Show the plot
        fignum: int, optional
            Define figure number. Note: Should only be used when plotting a singular case. 

        Returns:
        -------
        fig, ax - corresponds to generated figure
        '''

        if not fastout:
            try:
                fastout = self.fastout
            except:
                raise AttributeError('Cannot plot OpenFAST output data before it is loaded with load_fast_out.')
        if not cases:
            cases=self.plot_cases

        if averaging.lower() not in ['none', 'welch']:
            raise ValueError('{} is not a supported averaging method.'.format(averaging))

        if averaging_window.lower() not in ['hamming', 'hann', 'rectangular']:
            raise ValueError('{} is not a supported averaging window.'.format(averaging_window))

        fig, ax = plt.subplots(num=fignum)

        leg = []
        for channel, run in cases:
            try:
                # Find time
                Time = fastout[run]['Time']
                # Load PSD
                fq, y, info = spectral.fft_wrap(
                    Time, fastout[run][channel], averaging=averaging, averaging_window=averaging_window, detrend=detrend, nExp=nExp)
                # Plot data
                plt.loglog(fq, y, label='{}, run: {}'.format(channel, str(run)))
            except:
                print('{} is not an available channel in run {}'.format(channel, str(run)))
        # Show rotor speed range (1P, 3P, 6P?)
        if show_RtSpeed:
            if not RtSpeed_idx:
                RtSpeed_idx = [0]
                print('No rotor speed run indices defined, plotting spectral range for the first run only.')
            for rt_idx in RtSpeed_idx:
                f_1P_min = min(fastout[rt_idx]['RotSpeed']) / 60.  # Hz
                f_1P_max = max(fastout[rt_idx]['RotSpeed']) / 60.
                f_3P_min = f_1P_min*3
                f_3P_max = f_1P_max*3
                f_6P_min = f_1P_min*6
                f_6P_max = f_1P_max*6
                plt.axvspan(f_1P_min, f_1P_max, alpha=0.5, color=[0.7, 0.7, 0.7], label='1P')
                plt.axvspan(f_3P_min, f_3P_max, alpha=0.5, color=[0.8, 0.8, 0.8], label='3P')
                # if f_6P_min<1.:
                #     plt.axvspan(f_6P_min, f_6P_max, alpha=0.5, color=[0.9,0.9,0.9], label='6P')

        # Add specific frequencies if desired
        if add_freqs:
            co = np.linspace(0.3, 0.0, len(add_freqs))
            if add_freq_labels is None:
                add_freq_labels = [None]*len(add_freqs)
            for freq, flabel, c in zip(add_freqs, add_freq_labels, co):
                plt.axvline(freq, color=[c, c, c])  # , label=flabel)
                trans = transforms.blended_transform_factory(ax.transData, ax.transAxes)
                plt.text(freq+(10**np.floor(np.log10(freq))/10), 0.01, flabel, transform=trans)

        # Formatting
        plt.legend(loc='best')
        if len(list(cases)) > 1:
            plt.ylabel('PSD')
        else:
            unit_idx = fastout[run]['meta']['channels'].index(channel)
            plt.ylabel(
                'PSD ({}$^2$/Hz)'.format(fastout[run]['meta']['attribute_units'][unit_idx]))
        plt.xlabel('Frequency (Hz)')
        plt.grid(True)

        if showplot:
            plt.show()

        return fig, ax


def load_ascii_output(filename):
    '''
    Load FAST ascii output file 
    
    Parameters
    ----------
    filename : str
        filename
    
    Returns
    -------
    data : ndarray
        data values
    info : dict
        info containing:
            - name: filename
            - description: description of dataset
            - channels: list of attribute names
            - attribute_units: list of attribute units
    '''
    with open(filename) as f:
        info = {}
        info['name'] = os.path.splitext(os.path.basename(filename))[0]
        # Header is whatever is before the keyword `time`
        in_header = True
        header = []
        while in_header:
            l = f.readline()
            if not l:
                raise Exception('Error finding the end of FAST out file header. Keyword Time missing.')
            in_header= (l+' dummy').lower().split()[0] != 'time'
            if in_header:
                header.append(l)
            else:
                info['description'] = header
                info['channels'] = l.split()
                info['attribute_units'] = [unit[1:-1] for unit in f.readline().split()]

        # Read lines until a blank line (trailing comments may follow)
        lines = takewhile(lambda x: len(x.strip()) > 0, f.readlines())

        # Split each line into tokens, padding/truncating rows with wrong column count with nan
        n_cols = len(info['channels'])
        rows = [line.strip().split() for line in lines]
        rows = [row if len(row) == n_cols else row[:n_cols] + ['nan'] * (n_cols - len(row)) for row in rows]

        # Convert to numeric array
        data = np.array(rows, dtype=np.float64)
        return data, info


def load_binary_output(filename, use_buffer=True):
    """
            
    Info about ReadFASTbinary.m:
    
    Original Author: Bonnie Jonkman, National Renewable Energy Laboratory
    (c) 2012, National Renewable Energy Laboratory
    Edited for FAST v7.02.00b-bjj  22-Oct-2012
    03/09/15: Ported from ReadFASTbinary.m by Mads M Pedersen, DTU Wind
    10/24/18: Low memory/buffered version by E. Branlard, NREL
    18/01/19: New file format for exctended channels, by E. Branlard, NREL
    11/4/19: Implemented in ROSCO toolbox by N. Abbas, NREL
    8/6/20: Synced between rosco toolbox and weio by P Bortolotti, NREL

    Parameters
    ----------
    filename : str
        filename
    Returns
    -------
    data : ndarray
        data values
    info : dict
        info containing:
            - name: filename
            - description: description of dataset
            - channels: list of attribute names
            - attribute_units: list of attribute units
    """
    def fread(fid, n, type):
        fmt, nbytes = {'uint8': ('B', 1), 'int16':('h', 2), 'int32':('i', 4), 'float32':('f', 4), 'float64':('d', 8)}[type]
        return struct.unpack(fmt * n, fid.read(nbytes * n))

    def freadRowOrderTableBuffered(fid, n, type_in, nCols, nOff=0, type_out='float64'):
        """ 
        Reads of row-ordered table from a binary file.
        Read `n` data of type `type_in`, assumed to be a row ordered table of `nCols` columns.
        Memory usage is optimized by allocating the data only once.
        Buffered reading is done for improved performances (in particular for 32bit python)
        `nOff` allows for additional column space at the begining of the storage table.
        Typically, `nOff=1`, provides a column at the beginning to store the time vector.
        @author E.Branlard, NREL
        """
        fmt, nbytes = {'uint8': ('B', 1), 'int16':('h', 2), 'int32':('i', 4), 'float32':('f', 4), 'float64':('d', 8)}[type_in]
        nLines          = int(n/nCols)
        GoodBufferSize  = 4096*40
        nLinesPerBuffer = int(GoodBufferSize/nCols)
        BufferSize      = nCols * nLinesPerBuffer
        nBuffer         = int(n/BufferSize)
        # Allocation of data
        data = np.zeros((nLines,nCols+nOff), dtype = type_out)
        # Reading
        try:
            nIntRead   = 0
            nLinesRead = 0
            while nIntRead<n:
                nIntToRead = min(n-nIntRead, BufferSize)
                nLinesToRead = int(nIntToRead/nCols)
                Buffer = np.array(struct.unpack(fmt * nIntToRead, fid.read(nbytes * nIntToRead)))
                Buffer = Buffer.reshape(-1,nCols)
                data[ nLinesRead:(nLinesRead+nLinesToRead),  nOff:(nOff+nCols)  ] = Buffer
                nLinesRead = nLinesRead + nLinesToRead
                nIntRead   = nIntRead   + nIntToRead
        except:
            raise Exception('Read only %d of %d values in file:' % (nIntRead, n, filename))
        return data


    FileFmtID_WithTime              = 1 # File identifiers used in FAST
    FileFmtID_WithoutTime           = 2
    FileFmtID_NoCompressWithoutTime = 3
    FileFmtID_ChanLen_In            = 4

    with open(filename, 'rb') as fid:
        #----------------------------        
        # get the header information
        #----------------------------

        FileID = fread(fid, 1, 'int16')[0]  #;             % FAST output file format, INT(2)

        if FileID not in [FileFmtID_WithTime, FileFmtID_WithoutTime, FileFmtID_NoCompressWithoutTime, FileFmtID_ChanLen_In]:
            raise Exception('FileID not supported {}. Is it a FAST binary file?'.format(FileID))

        if FileID == FileFmtID_ChanLen_In: 
            LenName = fread(fid, 1, 'int16')[0] # Number of characters in channel names and units
        else:
            LenName = 10                    # Default number of characters per channel name

        NumOutChans = fread(fid, 1, 'int32')[0]  #;             % The number of output channels, INT(4)
        NT = fread(fid, 1, 'int32')[0]  #;             % The number of time steps, INT(4)

        if FileID == FileFmtID_WithTime:
            TimeScl = fread(fid, 1, 'float64')  #;           % The time slopes for scaling, REAL(8)
            TimeOff = fread(fid, 1, 'float64')  #;           % The time offsets for scaling, REAL(8)
        else:
            TimeOut1 = fread(fid, 1, 'float64')  #;           % The first time in the time series, REAL(8)
            TimeIncr = fread(fid, 1, 'float64')  #;           % The time increment, REAL(8)

        if FileID == FileFmtID_NoCompressWithoutTime:
            ColScl = np.ones ((NumOutChans, 1)) # The channel slopes for scaling, REAL(4)
            ColOff = np.zeros((NumOutChans, 1)) # The channel offsets for scaling, REAL(4)
        else:
            ColScl = fread(fid, NumOutChans, 'float32')  # The channel slopes for scaling, REAL(4)
            ColOff = fread(fid, NumOutChans, 'float32')  # The channel offsets for scaling, REAL(4)

        LenDesc      = fread(fid, 1, 'int32')[0]  #;  % The number of characters in the description string, INT(4)
        DescStrASCII = fread(fid, LenDesc, 'uint8')  #;  % DescStr converted to ASCII
        DescStr      = "".join(map(chr, DescStrASCII)).strip()

        ChanName = []  # initialize the ChanName cell array
        for iChan in range(NumOutChans + 1):
            ChanNameASCII = fread(fid, LenName, 'uint8')  #; % ChanName converted to numeric ASCII
            ChanName.append("".join(map(chr, ChanNameASCII)).strip())

        ChanUnit = []  # initialize the ChanUnit cell array
        for iChan in range(NumOutChans + 1):
            ChanUnitASCII = fread(fid, LenName, 'uint8')  #; % ChanUnit converted to numeric ASCII
            ChanUnit.append("".join(map(chr, ChanUnitASCII)).strip()[1:-1])

        # -------------------------
        #  get the channel time series
        # -------------------------

        nPts = NT * NumOutChans  #;           % number of data points in the file

        if FileID == FileFmtID_WithTime:
            PackedTime = fread(fid, NT, 'int32')  #; % read the time data
            cnt = len(PackedTime)
            if cnt < NT:
                raise Exception('Could not read entire %s file: read %d of %d time values' % (filename, cnt, NT))

        if use_buffer:
            # Reading data using buffers, and allowing an offset for time column (nOff=1)
            if FileID == FileFmtID_NoCompressWithoutTime:
                data = freadRowOrderTableBuffered(fid, nPts, 'float64', NumOutChans, nOff=1, type_out='float64')
            else:
                data = freadRowOrderTableBuffered(fid, nPts, 'int16', NumOutChans, nOff=1, type_out='float64')
        else:
            # NOTE: unpacking huge data not possible on 32bit machines
            if FileID == FileFmtID_NoCompressWithoutTime:
                PackedData = fread(fid, nPts, 'float64')  #; % read the channel data
            else:
                PackedData = fread(fid, nPts, 'int16')  #; % read the channel data

            cnt = len(PackedData)
            if cnt < nPts:
                raise Exception('Could not read entire %s file: read %d of %d values' % (filename, cnt, nPts))
            data = np.array(PackedData).reshape(NT, NumOutChans)
            del PackedData

    if FileID == FileFmtID_WithTime:
        time = (np.array(PackedTime) - TimeOff) / TimeScl;
    else:
        time = TimeOut1 + TimeIncr * np.arange(NT)

    # -------------------------
    #  Scale the packed binary to real data
    # -------------------------
    if use_buffer:
        # Scaling Data
        for iCol in range(NumOutChans):
            if np.isnan(ColScl[iCol]) and np.isnan(ColOff[iCol]):
                data[:,iCol+1] = 0 # probably due to a division by zero in Fortran
            else:
                data[:,iCol+1] = (data[:,iCol+1] - ColOff[iCol]) / ColScl[iCol]
        # Adding time column
        data[:,0] = time
    else:
        # NOTE: memory expensive due to time conversion, and concatenation
        data = (data - ColOff) / ColScl
        data = np.concatenate([time.reshape(NT, 1), data], 1)

    info = {'name': os.path.splitext(os.path.basename(filename))[0],
            'description': DescStr,
            'channels': ChanName,
            'attribute_units': ChanUnit}
    return data, info

def trim_output(fast_data, tmin=None, tmax=None, verbose=False):
    '''
    Trim loaded fast output data 
    Parameters
    ----------
    fast_data : list
        List of all output data from load_fast_out (list containing dictionaries)
    tmin : float, optional
        start time
    tmax : float, optional
        end time
    
    Returns
    -------
    fast_data : list
        list of dictionaries containing trimmed fast output data
    '''
    if isinstance(fast_data, dict):
        fast_data = [fast_data]



    # initial time array and associated index
    for fd in fast_data:
        if verbose:
            if tmin: 
                tmin_v = str(tmin) + ' seconds'
            else:  
                tmin_v = 'the beginning'
            if tmax: 
                tmax_v = str(tmax) + ' seconds'
            else: 
                tmax_v = 'the end'

            print('Trimming output data for {} from {} to {}.'.format(fd['meta']['name'], tmin_v, tmax_v))
        # Find time index range
        if tmin:
            T0ind = np.searchsorted(fd['Time'], tmin)
        else:
            T0ind = 0
        if tmax:
            Tfind = np.searchsorted(fd['Time'], tmax) + 1
        else: 
            Tfind = len(fd['Time'])

        if T0ind+1 > len(fd['Time']):
            raise ValueError('The initial time to trim {} to is after the end of the simulation.'.format(fd['meta']['name']))

        # # Modify time
        fd['Time'] = fd['Time'][T0ind:Tfind] - fd['Time'][T0ind]

        # Remove all vales in data where time is not in desired range
        for key in fd.keys():
            if key.lower() not in ['time', 'meta']:
                fd[key] = fd[key][T0ind:Tfind]


    return fast_data


if __name__ == "__main__":
    outfiles = [
        '/Users/dzalkind/Tools/ROSCO3/outputs/IPC_tune_1/NREL2p8/turb_bts/sweep_ipc_gains/NREL2p8_0.out',
        '/Users/dzalkind/Tools/ROSCO3/outputs/IPC_tune_1/NREL2p8/turb_bts/sweep_ipc_gains/NREL2p8_1.out',
        '/Users/dzalkind/Tools/ROSCO3/outputs/IPC_tune_1/NREL2p8/turb_bts/sweep_ipc_gains/NREL2p8_2.out',
        '/Users/dzalkind/Tools/ROSCO3/outputs/IPC_tune_1/NREL2p8/turb_bts/sweep_ipc_gains/NREL2p8_3.out'
    ]

    plt.rcParams["figure.figsize"] = [9,12]

    ROSCO = False
    ROSCO2 = False

    #  Define Plot cases 
    cases = {}
    cases['Gen. Speed Sigs.'] = ['Wind1VelX','BldPitch1', 'GenTq', 'GenSpeed','TwrBsMyt','GenPwr','RootMyb1']#,'PtfmPitch','PtfmYaw','NacYaw']

    op = output_processing()
    op_RO = output_processing()
    op_RO2 = output_processing()

    fast_out = []
    fast_out = op.load_fast_out(outfiles, tmin=0)
    if ROSCO:
        # Rosco outfiles
        r_outfiles = [out.split('.out')[0] + '.RO.dbg' for out in outfiles]
        rosco_out = op_RO.load_fast_out(r_outfiles, tmin=0)
        
        if ROSCO2:
            r_outfiles = [out.split('.out')[0] + '.RO.dbg2' for out in outfiles]
            rosco_out2 = op_RO2.load_fast_out(r_outfiles, tmin=0)
    
    # Combine outputs
    if ROSCO:
        comb_out = [None] * len(fast_out)
        for i, (r_out, f_out) in enumerate(zip(rosco_out,fast_out)):
            r_out.update(f_out)
            comb_out[i] = r_out
        if ROSCO2:
            for i, (r_out2, f_out) in enumerate(zip(rosco_out2,comb_out)):
                r_out2.update(f_out)
                comb_out[i] = r_out2
    else:
        comb_out = fast_out

        
    # Plot
    fig, ax = op.plot_fast_out(comb_out,cases, showplot=True)
