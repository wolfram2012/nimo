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

from common import CorrelationFilter

import pyvision as pv
import numpy as np

'''Minimum Average Correlation Energy Filters'''

##
# Minimum Average Correlation Energy Filters
#
# <p> This class is used to train MACE, SDF, and OTF filters.  It includes a 
# parameter alpha which can be used to tune the algorthm.  Alpha of 1.0 
# (default) is a standard MACE implementation.  Alpha of 0.0 computes SDF. 
# Anything in between is an variant of optimal tradeoff filter assuming
# white noise.
#
# A. Mahalanobis, B.V.K. Vijaya Kumar, and D. Casasent, 
# "Minimum average correlation energy filters," Appl. Opt. 26, pp. 3633-3630 (1987)
#    
# B.V.K. Vijaya Kumar, "Minimum Variance synthetic discriminant functions," J. Opt. Soc. Am A, 
# Vol 3, pp1579-1584. 
class MACE(CorrelationFilter):
    
    def __init__(self,tile_size,alpha=1.0,**kwargs):
        ''' 
        alpha = 1.0 (MACE)
        alpha = 0.0 (SDF)
        0.0 < alpha < 1.0 (OTF)
        TODO: Recheck alpha value facts
        '''
        self.alpha = alpha
        self.X = []
        self.u = []
        self.D = None
        self.n = 0.0
        self.tile_size = tile_size
        self.trained = False

        CorrelationFilter.__init__(self,tile_size,**kwargs)

        
    def addTraining(self,val,im,ilog=None): 
        ''' TODO: '''
        if self.tile_size == None:
            self.tile_size = im.size
            
        mat = self.preprocess(im)
        if ilog: ilog.log(pv.Image(mat),label="OTF_TRAINING")
        mat = np.fft.fft2(mat)
        self.X.append(mat.flatten())
        self.u.append(float(val))
        psd = abs(mat.flatten())**2
        if self.D == None:
            self.D = psd
        else:
            self.D += psd
            
        self.n += 1

        
    def train(self,ilog=None):
        X = np.array(self.X).transpose()
        u = np.array(self.u).reshape(len(self.u),1)
        del self.X
        del self.u
        D = self.D/self.n
        assert self.alpha >= 0.0
        assert self.alpha <= 1.0
        D = self.alpha*D + np.sqrt(1-self.alpha**2)*np.ones((len(D),)) # OTF
        
        Dinv = D**-1 
        Dinv = np.eye(Dinv.shape[0])*Dinv.reshape(Dinv.shape[0],1)
        
        tmp1 = np.dot(np.dot(X.conj().transpose(),Dinv),X)
        tmp1 = np.linalg.pinv(tmp1)
        tmp2 = np.dot(Dinv,np.dot(X,tmp1))
        H = np.dot(tmp2,u).reshape(self.tile_size)
        #self.filter = H
        
        # Recenter filter at pixel 0,0
        self.filter = np.fft.fft2(np.fft.fftshift(np.fft.ifft2(H.conj())))
        self.trained = True

        if ilog != None:
            ilog(self.asImage(),"MACE")



class UMACE(CorrelationFilter):
    
    def __init__(self,tile_size,alpha=1.0,**kwargs):
        ''' 
        alpha = 1.0 (MACE)
        alpha = 0.0 (SDF)
        0.0 < alpha < 1.0 (OTF)
        TODO: Recheck alpha value facts
        '''
        self.alpha = alpha
        self.m = None
        self.D = None
        self.n = 0.0
        self.tile_size = tile_size
        self.trained = False

        CorrelationFilter.__init__(self,tile_size,**kwargs)

        
    def addTraining(self,val,im,ilog=None): 
        ''' TODO: '''
        if self.tile_size == None:
            self.tile_size = im.size
            
        mat = self.preprocess(im)
        mat = np.fft.fft2(mat)

        if self.m == None:
            self.m = mat.flatten()
        else:
            self.m += mat.flatten()

        psd = abs(mat.flatten())**2
        if self.D == None:
            self.D = psd
        else:
            self.D += psd
            
        self.n += 1
        
        if ilog: ilog.log(pv.Image(mat.real),label="UMACE_TRAINING")

        
    def train(self,ilog=None):
        m = self.m/self.n
        D = self.D/self.n
        assert self.alpha >= 0.0
        assert self.alpha <= 1.0
        D = self.alpha*D + np.sqrt(1-self.alpha**2)*np.ones((len(D),)) # OTF
        
        Dinv = D**-1 
        
        H = (Dinv*m).reshape(self.tile_size)
        self.filter = np.fft.fft2(np.fft.fftshift(np.fft.ifft2(H.conj())))
        #self.filter = np.fft.ifft2(H).real
        self.trained = True
        
        if ilog != None:
            ilog(self.asImage(),"UMACE")



