# Copyright 2010, David S. Bolme, Colorado State University Research 
# Foundation
#
# Colorado State University Software Evaluation License Agreement
#
# This license agreement ("License"), effective today, is made by and between 
# you (hereinafter referred to as the "Licensee") and the Board of Governors of 
# the Colorado State University System acting by and through Colorado State 
# University, an institution of higher education of the State of Colorado, 
# located at Fort Collins, Colorado, 80523-2002 ("CSU"), and concerns certain 
# software described as "Correlation Filters for Detection, Recognition, and 
# Registration," a system of software programs for advanced video signal 
# processing and analysis. 
#
# 1.  General. A non-exclusive, nontransferable, perpetual license is granted 
#     to the Licensee to install and use the Software for academic, non-profit, 
#     or government-sponsored research purposes. Use of the Software under this 
#     License is restricted to non-commercial purposes. Commercial use of the 
#     Software requires a separately executed written license agreement. 
#    
# 2.  Permitted Use and Restrictions. Licensee agrees that it will use the 
#     Software, and any modifications, improvements, or derivatives to the 
#     Software that the Licensee may create (collectively, "Improvements") 
#     solely for internal, non-commercial purposes and shall not distribute, 
#     transfer, deploy, or externally expose the Software or Improvements to any 
#     person or third parties without prior written permission from CSU. The 
#     term "non-commercial," as used in this License, means academic or other 
#     scholarly research which (a) is not undertaken for profit, or (b) is not 
#     intended to produce works, services, or data for commercial use, or (c) is 
#     neither conducted, nor funded, by a person or an entity engaged in the 
#     commercial use, application or exploitation of works similar to the 
#     Software. 
#    
# 3.  Ownership and Assignment of Copyright. The Licensee acknowledges that 
#     CSU has the right to offer this copyright in the Software and associated 
#     documentation only to the extent described herein, and the offered 
#     Software and associated documentation are the property of CSU. The 
#     Licensee agrees that any Improvements made by Licensee shall be subject 
#     to the same terms and conditions as the Software. Licensee agrees not to 
#     assert a claim of infringement in Licensee copyrights in Improvements in 
#     the event CSU prepares substantially similar modifications or derivative 
#     works. The Licensee agrees to use his/her reasonable best efforts to 
#     protect the contents of the Software and to prevent unauthorized 
#     disclosure by its agents, officers, employees, and consultants. If the 
#     Licensee receives a request to furnish all or any portion of the Software 
#     to a third party, Licensee will not fulfill such a request but will refer 
#     the third party to the CSU Computer Science website 
#     http://www.cs.colostate.edu/~vision/ocof.html so that the third party's 
#     use of this Software will be subject to the terms and conditions of this 
#     License. Notwithstanding the above, Licensee may disclose any 
#     Improvements that do not involve disclosure of the Software. 
#     
# 4.  Copies. The Licensee may make a reasonable number of copies of the 
#     Software for the purposes of backup, maintenance of the Software or the 
#     development of derivative works based on the Software. These additional 
#     copies shall carry the copyright notice and shall be controlled by this 
#     License, and will be destroyed along with the original by the Licensee 
#     upon termination of the License. 
#     
# 5.  Acknowledgement. Licensee agrees that any publication of results obtained 
#     with the Software will acknowledge its use by an appropriate citation as 
#     specified in the documentation. 
#     
# 6.  Acknowledgment. CSU acknowledges that the Software executes patent pending 
#     Algorithms for the purpose of non-commercial uses. Licensee acknowledges 
#     that this Agreement does not constitute a commercial license to the 
#     Algorithms. Licensee may apply for a commercial license to the Algorithms 
#     from Colorado State University Research Foundation by visiting 
#     http://www.csuventures.org. 
#    
# 7.  Disclaimer of Warranties and Limitation of Liability. THE SOFTWARE IS 
#     PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
#     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
#     AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. CSU MAKES NO 
#     REPRESENTATION OR WARRANTY THAT THE SOFTWARE WILL NOT INFRINGE ANY PATENT 
#     OR OTHER PROPRIETARY RIGHT. IN NO EVENT SHALL CSU BE LIABLE FOR ANY 
#     DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
#     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
#     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
#     OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
#     SUCH DAMAGE. 
#     
# 8.  Termination. This License is effective until terminated by either party. 
#     Your rights under this License will terminate automatically without notice 
#     from CSU if you fail to comply with any term(s) of this License. Upon 
#     termination of this License, you shall immediately discontinue all use of 
#     the Software and destroy the original and all copies, full or partial, of 
#     the Software, including any modifications or derivative works, and 
#     associated documentation. 
#    
# 9.  Governing Law and General Provisions. This License shall be governed by 
#     the laws of the State of Colorado, excluding the application of its 
#     conflicts of law rules. This License shall not be governed by the United 
#     Nations Convention on Contracts for the International Sale of Goods, the 
#     application of which is expressly excluded. If any provisions of this 
#     License are held invalid or unenforceable for any reason, the remaining 
#     provisions shall remain in full force and effect. This License is binding 
#     upon any heirs and assigns of the Licensee. The License granted to 
#     Licensee hereunder may not be assigned or transferred to any other person 
#     or entity without the express consent of the Regents. This License 
#     constitutes the entire agreement between the parties with respect to the 
#     use of the Software licensed hereunder and supersedes all other previous 
#     or contemporaneous agreements or understandings between the parties, 
#     whether verbal or written, concerning the subject matter. 

'''
This

Created on Apr 18, 2010

@author: bolme
'''

import numpy as np
import pyvision as pv
import scipy as sp
import scipy.ndimage

class CorrelationFilter:
    ''' This class implements many methods and interfaces that are common to all filters. '''

    def __init__(self, size, norm_log=False, norm_meanunit=True, norm_window = pv.cosineWindow, bbox=None, ilog=None):
        '''Initial some of the basic elements of the correlation filter.'''
        
        # Set the size of this filter
        self.size = size
        
        # Setup Normalization Steps
        self.norm_log = norm_log
        self.norm_meanunit = norm_meanunit
        
        self.norm_window = norm_window
            
        # Only search for peaks in this bounding box
        self.bbox = bbox
        
        # A member variable for the filter stored in the fourier domain
        self.filter = None
        
        # Caches used for resized versions of the filter and windows
        self.window_cache = {}
        self.filter_cache = {}
        
        # Fast Local Maximum Detector (Created when first needed)
        self.maximum_detector = None

        
    
    def preprocess(self, tile, ilog=None):
        ''' Implements some standard preprocessing for all filters. '''
        
        # Get the image tile as a numpy matrix.
        if isinstance(tile,pv.Image):
            mat = tile.asMatrix2D()
        else:
            # Assume it is already a matrix
            mat = tile
        
        # Apply a logarithmic transformation to pixel values.
        if self.norm_log:
            mat = np.log(mat + 1)
            
        # print "tile is:",mat
        # Transform pixel values to have a mean of zero and unit length
        if self.norm_meanunit:
            mat = pv.meanUnit(mat)
        # print "mat is:",mat

            
        # Window the input tile to reduce fourier edge effects.
        window = self._resizeWindow(mat.shape)
        if window is not None:
            mat = mat * window
            
        if ilog != None:
            ilog(pv.Image(mat),label="PREPROCESSED")
            
        return mat
        
        
    def addTraining(self, tile, output, ilog=None):
        ''' Add training data.  This method should be overridden by sub classes.  Subclasses should compute the filter and assign the Fourier domain filter to the member variable self.filter'''
        raise NotImplementedError()

    
    def train(self,ilog=None):
        '''This should be overridden by subclasses. Subclasses should compute the filter and assign the Fourier domain filter to the member variable self.filter'''
        raise NotImplementedError()
        
    
    def correlate(self, tile, F=None, phase_only=False, ilog=None):
        ''''''
        
        # Correlate with the filter
        if F == None:
            # Preprocess the image
            mat = self.preprocess(tile,ilog=ilog)

            F = np.fft.fft2(mat)
            
        G = self._resizeFilter(F.shape) * F
        if phase_only:
            G = G/np.abs(G)
        g = np.fft.ifft2(G)
        
        # Return just the real part
        return g.real
        

    def locate(self, tile, corr=None, subpixel_est=True, bbox=None, ilog=None):
        ''''''
        if corr == None:
            corr = self.correlate(tile,ilog=ilog)
            
        if bbox == None:
            bbox = self.bbox
            
        if bbox:        
            #print "CorrShape:",corr.shape
            idx = corr[bbox[0]:bbox[1],bbox[2]:bbox[3]].argmax()
            _,h = corr[bbox[0]:bbox[1],bbox[2]:bbox[3]].shape
            x = bbox[0]+idx/h
            y = bbox[2]+idx%h
        else:
            idx = corr.argmax()
            _,h = corr.shape        
            x = idx/h
            y = idx%h
            
        if subpixel_est:
            dx,dy = subpixel(corr[x-3:x+4,y-3:y+4])
            x += dx
            y += dy
            
        return pv.Point(x,y)
    
    
    def detect(self,tile,corr=None,compute_psrs=False,ilog=None):
        if corr == None:
            corr = self.correlate(tile,ilog=ilog)
        
        # Create the maximum detector the first time it is needed
        if self.maximum_detector == None: 
            self.maximum_detector = pv.LocalMaximumDetector()
        
        # Find local maximums in the filter response
        points, values = self.maximum_detector(corr,threshold=0.0)

        if compute_psrs:        
            # Compute psrs
            w,h = corr.shape
            corrf = corr.flatten()
    
            psrs = []
            for x,y in points:
                pk = corr[x,y]
            
                # Mask out the sidelobe
                mask = np.zeros([w,h],dtype=np.bool)
                mask[x-10:x+11,y-10:y+11] = True
                mask[x-5:x+6,y-5:y+6] = False
                mask = mask.flatten()
                sidelobe = corrf[mask]
            
                # compute the psr
                mn = sidelobe.mean()
                sd = sidelobe.std()
                psr = (pk-mn)/sd
                
                if np.isnan(psr):
                    psrs.append(0)
                else:
                    psrs.append(psr)
    
            
        points = [pv.Point(x,y) for x,y in points]
    
        if compute_psrs:
            return np.array(points), values, np.array(psrs)
        return np.array(points), values
        
            


    
    def psr(self, tile, corr=None, ilog=None):
        '''
        Compute the peak to sidelobe ratio.  This is a good measure of quality.
        '''
        if corr == None:
            corr = self.correlate(tile,ilog=ilog)
            
        rows,cols = corr.shape

        # Find the peak
        i = corr.argmax()
        x,y = i/cols, i%cols
        corr = corr.flatten()
        pk = corr[i]
        
        # Mask out the sidelobe
        mask = np.ones([rows,cols],dtype=np.bool)
        mask[x-5:x+6,y-5:y+6] = False
        mask = mask.flatten()
        sidelobe = corr[mask]
        
        # compute the psr
        mn = sidelobe.mean()
        sd = sidelobe.std()
        return (pk-mn)/sd

    
    def asImage(self, ilog=None):
        ''''''
        mat = np.fft.ifft2(self.filter.conj())
        mat = np.fft.fftshift(mat)
        return pv.Image(mat.real)
    
    def asSpectrum(self):
        return self.filter*self.filter.conj()

    def scale(self,scale,order=3):
        '''
        Create a scaled copy of the correlation filter.
        
        @param scale: a scale factor.
        @param order: the order of spline interpolation.
        
        @returns: A rescaled correlation filter
        @rtype: ocof.CorrelationFilter
        '''
        filter = np.fft.ifft2(self.filter)
        filter = sp.ndimage.zoom(filter,scale,order=order)
        result = CorrelationFilter(self.size, norm_log=self.norm_log, norm_meanunit=self.norm_meanunit, norm_window = self.norm_window, bbox=self.bbox)
        result.filter = np.fft.fft2(filter)
        return result
    
    
    def _resizeFilter(self,size):
        '''
        Resize the filter 
        '''
        if size == self.filter.shape:
            return self.filter
        
        if not self.filter_cache.has_key(size):
            filter = np.fft.ifft2(self.filter)
            w,h = size
            
            fw,fh = filter.shape
            tmp = np.zeros((w,h), np.complex128) #TODO: check this
            
            w = min(w,fw)
            h = min(h,fh)
            
            tmp[ :w/2, :h/2] = filter[ :w/2, :h/2]
            tmp[ :w/2,-h/2:] = filter[ :w/2,-h/2:]
            tmp[-w/2:,-h/2:] = filter[-w/2:,-h/2:]
            tmp[-w/2:, :h/2] = filter[-w/2:, :h/2]
            
            self.filter_cache[size] = np.fft.fft2(tmp)
        
        return self.filter_cache[size]

    
    def _resizeWindow(self,size):
        if self.norm_window == None:
            return None
        
        if not self.window_cache.has_key(size):
            window = self.norm_window(size)
            self.window_cache[size] = window
        
        return self.window_cache[size]

    
    
def subpixel(b):
    # TODO: Locations may always be estimated low

    w,h = b.shape
    x = np.arange(w).reshape((w,1)) * np.ones((1,h)) - w/2
    y = np.arange(h).reshape((1,h)) * np.ones((w,1)) - w/2

    x = x.flatten()
    y = y.flatten()
    c = np.ones((w,h)).flatten()
    A = np.array([x*x,x,y*y,y,c]).transpose()
    b = b.flatten()

    try:
        coef = np.linalg.lstsq(A,b)
    except:
        return 0.0,0.0

    a,b,c,d,_ = coef[0]
    
    x = -b/a
    y = -d/c
    
    if x > 1 or x < -1 or y > 1 or y < -1:
        return 0.0,0.0
    
    return x,y

    
def createPointTarget(x,y,size,sigma=2.0,**kwargs):
    '''Create a target with a Gaussian shaped peak of 1.0 at x,y and is 0.0 everywhere else.'''
    x = np.arange(size[0])-x
    y = np.arange(size[1])-y
    scale = 1.0/(sigma*sigma)
    target = np.exp(-scale*x*x).reshape(size[0],1)*np.exp(-scale*y*y).reshape(1,size[1])
    return target


def createDeltaTarget(x,y,size,**kwargs):
    '''Create a target that has a value of 1.0 at x,y and is 0.0 everywhere else.'''
    x = int(round(x))
    y = int(round(y))
    target = np.zeros(size,dtype=np.float64)
    target[x,y] = 1.0
    return target


