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
This module contains the L{MOSSETrack} class which is the standard 
implementation of a MOSSE filter based tracking system.  For usage
instructions please see the L{MOSSETrack} documentation.  Supporting
definitions are also found here.

Created on Apr 23, 2010

@author: bolme
'''

import ocof
import ocof.filters.common as common
import numpy as np
import pyvision as pv
import time
import copy
import cv2

STATUS_INITIALIZING = "INITAILIZING"
'''Status for initializing.'''
STATUS_GOOD = "TRACKING"
'''Status for tracking.'''
STATUS_DROPPED = "DROPPED"
'''Status for a dropped track.'''


INIT_ONE_TRANSFORM = "INIT_ONE_TRANSFORM"
'''Initialize using just a single input'''

INIT_NINE_TRANSFORM = "INIT_NINE_TRANSFORM"
'''Initialize using nine transforms in rotation and scale.'''

class MOSSETrack:
    '''
    Visual object tracking using a MOSSE Filter.  This documentation describes
    how to use this software and achieve good tracking performance. A 
    publication describing the tracking algorithm can be found here:
    
    D. S. Bolme, J. R. Beveridge, B. A. Draper, and Y. M. Lui. Visual Object 
    Tracking using Adaptive Correlation Filters. Computer Vision and Pattern 
    Recognition. June 2010.
    
    U{http://www.cs.colostate.edu/~bolme/publications/Bolme2010Tracking.pdf}
    
    Using the Tracker
    =================
      I{Note: A working example of tracking can be found in the C{src/examples/}
      directory.}
    
      Tracking is initialized using a frame of the video and a rectangle which 
      specifies the target to be tracked.  The interface to the tracker is 
      intended to be  simple to use, but there are still many optional 
      configuration options that allows tracking to be tuned to different 
      scenarios.
      
      The first step is to import the pyvision and ocof libraries::
      
        import pyvision as pv
        import ocof
    
      A track is initialized on creation by providing a frame (pv.Image) and a rectangle (pv.Rect), which 
      specifies the target to be tracked::
    
        track = ocof.MOSSETrack(frame,rect)
    
      For each additional from the track is updated by calling the method L{update}::
    
        track.update(frame)
      
    Using the Tracking Output
    =========================
      To find the location of the track in each following frame the following
      methods can be called to get points and rectangles in PyVision format::
      
        pt = track.asPoint()
        rect = track.asRect()
        
      These values can then be rendered on the frame or used by following
      processing steps::

        frame.annotatePoint(pt)
        frame.annotateRect(rect)
    
      As a convenience there is also a method that will annotate a frame with 
      diagnostics information such as timing data, the PSR, track location,
      and images of the input, filter, and output.  This additional information
      can prove valuable in understanding and correcting challenges encountered
      by the tracker::
      
        track.annotateFrame(frame)
        
      One of the most common things to do with the output of the tracker is
      to obtain a cropped tile representing the tracking rectangle.  This can 
      be done easily using the PyVision affine transform utilities.  In this
      example, C{crop} is an instance of an affine transform class that can be
      called like a function on images and points::
        
        crop = pv.AffineFromRect(rect,(64,64))
        tile = crop(frame)
        

    Tips, Tricks, and Performance Tuning
    ====================================
      This section will cover some tips and tricks for obtaining good 
      performance with the tracker.  While MOSSE based visual tracking is one
      of the best algorithms available, there are some simple thing that can
      be done in order to achieve the best tracking performance.
      
      B{Know Your Target}
      
      Before tracking it is important to understand the characteristics of the
      target.  It is a good idea to answer these questions.
      
        1. How quickly does the target change its location in the frame?
        2. How quickly does a target change its appearance due to scale 
           changes, rotations, or deformations?
        3. What parts of the target have the most consistent appearance?
        4. What external factors may cause tracking difficulties? Lighting? 
           Shadows? Occlusions? etc.
        5. What are the highest contrast "edges" within the target or on its 
           boundary? 
        6. How will the target move?  Will it change size as it gets closer or
           further from the camera?  Does the target perform out-of-plane 
           rotations? Does the target perform in-plane rotations? 

      First, consider these two target types: people and vehicles.  People are 
      very difficult targets to track for a few reasons.  The biggest reason for
      this is because the persons appearance changes as they move.  As a person
      walks the swing there arms and legs which makes those body parts useless 
      for visual tracking.  Additionally, a person can turn very quickly.  This
      means that one moment the camera could be viewing the front of a person 
      a few frames later it may be viewing the back of the person.  In order to
      track a person, the tracker will need to adapt quickly to the changes in
      appearance. 
      
      A vehicle, on the other hand, is one of the easiest targets to track.  
      This is because the vehicle is a ridged object with no moving parts. They
      also turn slowly taking at least 3 to 5 seconds to make a 90 degree turn. 
      Because vehicles do not go through many difficult appearance changes 
      The tracker can adapt slower which reduces track drift and the filter
      can learn a more consistent appearance model of the vehicle.
      
      One of the most difficult things for the tracker to handle is out-of-plain
      rotations.  This is because the MOSSE Tracker will lock on to the center
      point of the tracking rectangle.  Consider tracking a face where center 
      of the track is initialized on the nose.  As long as the face is looking 
      directly at the camera, the tracker will keep the rectangle centered on 
      the nose.  If the the face turns to the left or right so that a profile
      view is shown, the track will still be centered on the same point on the
      nose.  This is a problem because in the profile view half of the tracking 
      rectangle will be occupied by the background.  The tracker might then 
      adapt to track features of the background instead of the face.    

      B{Initialization Parameters}
      
      There are many optional parameters that can be passed to the C{MOSSETrack} 
      constructor; however, the initial rectangle (C{rect}), the frame rate, C{update_rate}, 
      and C{min_psr} seem to have the most effect on performance.  It is 
      recommended that those parameters should be adjusted first.  
      Documentation on other parameters can be found with the L{constructor<__init__>}.
      
      B{Selecting the Tracking Rectangle} 
      
      One of the biggest factors that effects the performance of the tracker
      is the selection of the tracking rectangle. When tracking, the MOSSE 
      filter will automatically use the highest contrast and most stable edges 
      in the rectangle for tracking. For this reason it is important that the 
      tracking rectangle capture those features.  
      
      As a general rule, tracking works best when the tracking rectangle is 
      approximately square even if the target is not. Additionally, the 
      targets boundary is often one of the best features to track.  Therefore,
      it is good practice for the tracking rectangle to be 15% to 25% larger
      than the target being tracked.
      
      When tracking people this is especially important.  Because the legs 
      move quickly it is often unnecessary to include them in the tracking 
      rectangle.  In practice, the tracker works best when it is initialized
      using a square that is centered on the torso and the size is set such 
      that the square extends to about the knees.  
      
      It is also important that the tracking rectangle be adjusted to account
      for the speed of the target.  A good rule of thumb is that the target
      targets velocity per frame should be about one tenth the width
      of the tracking rectangle. That means if a target moves 10 pixels per 
      frame the rectangle should have dimensions of 100x100 pixels or larger.
      
      B{Frame Rates}
      
      Tracking is easier if the target changes very little per frame.  That 
      means that faster frame rates means better tracking.  For most scenes
      the tracker does well at 10 to 15 fps however 30 fps or better is 
      preferred.
       
      B{Update Rate}
      
      The C{update_rate} controls how fast the tracker adapts to changing 
      appearance of the target.  Typically, the update rate should be set
      somewhere between 0.0 and 0.25.  A value of 0.0 does not allow the 
      filter to adapt at all, while 1.0 would cause the filters appearance 
      to be based solely upon the last frame processed.  For fast changing 
      targets such as football players dodging in and out of traffic the 
      update rate should closer to 0.25 while slowly changing targets such
      as cars should use values closer to 0.03. By default this value is
      set to 0.0625 which works well in many scenarios. The update rate is 
      related to the frame rate in that slower frame will probably require
      faster update rates.
      
      B{Occlusion Detection}
      
      This tracker use the Peak-to-Sidelobe Ratio (PSR) as a way to measure
      track quality and to detect occlusion.  PSR is a measure of the 
      correlation peak strength where higher values indicate more confidence
      that the tracker is operating properly.  When occlusion is detected when
      the PSR drops below a threshold.  This puts the tracker in a state where
      the filter updates and location updates stop and the tracker is waiting 
      for the target to reappear.  If the target reappears and the PSR exceeds
      the threshold then tracking will continue.
      
      The C{min_psr} controls the threshold at which occlusion is detected.
      This is typically set between 6.0 and 10.0 where higher values will
      error on the side rejecting a valid track as a failure and lower values 
      will maintain weak tracks but will be more likely to allow tracking 
      failures and occlusions to go undetected. If this occlusion detection is 
      undesired setting C{min_psr} to 0.0 will disable this feature. 
      
      I{Note: Because the tracker stops updates during occlusion the tracking 
      rectangle will not move.  If the target reappears outside of this 
      rectangle it will not be reaquired.  If the camera is stationary and
      the target moves behind an obstruction this means that the tracking 
      rectangle will not be centered over the target.  It is possible to adjust
      the rectangle center to follow the predicted path of the target 
      using the L{setCenter} method and therefore reaquire targets in this
      scenario.}
    '''
    
    def __init__(self,
                 frame,
                 rect,
                 update_rate = 1.0/16.0, 
                 min_psr = 6.0,
                 max_psr=None,
                 tile_size=(64,64), 
                 reg = 0.1, 
                 type = "MOSSE", 
                 sigma=2.0, 
                 init_time = 8,
                 strategy=INIT_NINE_TRANSFORM,
                 dscale=0.05,
                 drotate=0.1, 
                 update_location=True,
                 subpixel=False,
                 color="red", 
                 **kwargs):
        '''
        This function creates and initializes the filter based tracking.  On 
        creation the tracker requires two argument.  The first frame for 
        tracking, and a rectangle specifying the tracked object.
        
        @param frame: A source video frame to _initialize from.
        @type  frame: pv.Image
        @param rect: A rectangle defining the object to be tracked in "frame".
        @type  rect: pv.Rect         
        @keyword update_rate: The adaptation rate of the filter.
        @type  update_rate: 0.0 <= float <= 1.0
        @keyword tile_size: The down sampled size for the track window. 
        @type  tile_size: (width,height)
        @keyword sigma: The radius of the Gaussian used to generate the filter output.
        @type  sigma: float
        @keyword reg: A regularization parameter to improve the stability of the filter.
        @type  reg: float
        @keyword type: The type of filter used for tracking.
        @type  type: "MOSSE","ASEF","UMACE", or "MEAN"
        @keyword strategy: a strategy used to create the first filter used by the tracker.
        @type  strategy: INIT_ONE_TRANSFORM, INIT_NINE_TRANSFORM
        @keyword dscale: The size of the scale changes for initialization.
        @type  dscale: float
        @keyword drotate: The size of the rotation change for initialization (in radians).
        @type  drotate: float
        @keyword min_psr: Minimum PSR needed for tracking.  The threshold for occlusion detection.
        @type  min_psr: float
        @keyword max_psr: The threshold for turning off adaptation.  This may computation time when the tracker is performing well.  Set to a large value probably greater than 30.
        @type  max_psr: float
        @keyword init_time: The minimum number for frames before min_psr threshold applies.  Provides a short initialization time. 
        @type  init_time: int
        @keyword update_location: Disables location update for track.  Good for things like measuring camera motion.
        @type  update_location: Boolean
        @keyword subpixel: Allow subpixel estimation of track center.  This may increase instability.
        @type  subpixel: boolean
        @keyword color: A color used for annotateFrame.
        @type  color: "red" or "#FF0000"
        @param kwargs: Additional keyword parameters passed to the filter.
        @type  kwargs: dict
        '''
        # Set Defaults

        # Setup translation filters for fast peak creation
        ocof.translationFilter.initSize(tile_size)

        self.rect = None
        self.psr_cache = None
        self.corr = None
        self.min_psr = min_psr
        self.max_psr = max_psr
        self.init_time = init_time
        self.life = 0
        
        
        self.tile_size = tile_size
        self.filter = _TrackingFilter(tile_size,type=type,sigma=sigma,reg=reg,norm_meanunit=True,**kwargs)
        # self.resampled = cv.CreateImage(self.tile_size,cv.IPL_DEPTH_8U,3)
        self.prevRect = None
        self.update_rate = update_rate

        self.frame = -1
        self.update_time = 0.0
        
        self.strategy = strategy
        self.dscale = dscale
        self.drotate = drotate
        
        self.update_location = update_location
        self.subpixel = subpixel

        self.best_estimate = None
        
        self._initialize(frame,rect)
        
        self.in_bounds = True
        
        
    def getStatus(self):
        '''
        Returns a code indicating the trackers evaluation of its own status.
          - B{STATUS_INITIALIZING} - Indicates the tracker was just initialized 
            and is ready to track.
          - B{STATUS_GOOD} - The tracker is tracking properly (psr > min_psr).
          - B{STATUS_DROPPED} - The psr indicates that the track has dropped or 
            the target is occluded.  The track is in a state waiting for the target
            to reappear.
            
        @return: A status code.
        @rtype: str
        '''
        psr = self.psr()
        if psr == 0:
            return STATUS_INITIALIZING
        elif psr > self.min_psr:
            return STATUS_GOOD
        else:
            return STATUS_DROPPED

    def _initialize(self,frame,rect):
        '''
        A private method that initializes the filter and prepares this 
        instance of a MOSSETrack to receive new frames.
        
        @param frame: A source video frame to _initialize from.
        @type frame: pv.Image
        @param rect: A rectangle defining the object to be tracked in "frame".
        @type rect: pv.Rect 
        '''
        start = time.time()
        self.rect = copy.deepcopy(rect)
        affine = pv.AffineFromRect(rect,self.tile_size)
        pt = affine.transformPoint(self.rect.center())
        if self.strategy == INIT_ONE_TRANSFORM:
            tmp = affine.transformImage(frame)
            self.filter.addTraining(tmp,pt)
            self.input = tmp
        elif self.strategy == INIT_NINE_TRANSFORM:
            for scale in [-1,0,1]:
                scale = pv.AffineRotate(scale*self.dscale,self.tile_size,center=pt)
                for rotate in [-1,0,1]:
                    rot = pv.AffineRotate(rotate*self.drotate,self.tile_size,center=pt)
                    pt = (scale*rot*affine).transformPoint(self.rect.center())
                    tmp = (scale*rot*affine).transformImage(frame)
                    self.filter.addTraining(tmp,pt)
            self.input = tmp
        self.corr = np.zeros(self.tile_size,dtype=np.float64)
        self.frame = 0
        stop = time.time()
        self.update_time = stop-start
        self.best_estimate = rect.center()
        
    def setCenter(self,point):
        '''
        Used to adjust the center point of the track.  
        
        @param point: The new center for the track.
        @type point:  pv.Point
        '''
        #self.center = point
        x = point.X()
        y = point.Y()
        w = self.rect.w
        h = self.rect.h
        self.rect = pv.CenteredRect(x,y,w,h)
        
    def setSize(self,size):
        '''
        Used to adjust the size of the tracking rect.
        
        @param size: (width,height)
        @type size: (float,float)
        '''
        x = self.rect.center().X()
        y = self.rect.center().Y()
        w,h = size
        self.rect = pv.CenteredRect(x,y,w,h)
        
    def update(self,frame):
        '''
        This is the main work function for the tracker.  After initialization, 
        this function should be called on each new frame.  This function:
        
            1. Extracts the tracking window from the new frame.
            2. Applies the filter to locate the new center of the target.
            3. Updates the filter (if PSR exceeds the threshold).
            4. Updates the internal state of the tracker to reflect the new location and status.
            
        @param frame: a new frame from the video.
        @type frame: pv.Image 
        '''
        
        start = time.time()
        
        self.frame += 1

        self.psr_cache = None
        
        tile,affine = frame.crop(self.rect,size=self.tile_size,return_affine=True)
        #affine = pv.AffineFromRect(self.rect,self.tile_size)
        self.prevRect=copy.deepcopy(self.rect)
        
        #tile = affine.transformImage(frame)
        self.input = tile
        
        corr = self.filter.correlate(tile)
        self.corr = corr
        
        # Find the peak
        _,cols = self.corr.shape
        i = self.corr.argmax()
        x,y = i/cols, i%cols
        target = pv.Point(x,y)
        
        self.best_estimate = affine.invert(target)
        
        if self.subpixel:
            dx,dy = common.subpixel(self.corr[x-3:x+4,y-3:y+4])
            target = pv.Point(x+dx, y+dy)
        
        # check status
        target_rect = pv.CenteredRect(target.X(),target.Y(),self.rect.w,self.rect.h)
        frame_rect = pv.Rect(0,0,frame.size[0],frame.size[1])
        self.in_bounds = frame_rect.containsRect(target_rect)
        status = self.getStatus()
        
        # Status == Good: Update the filter
        psr = self.psr()
        if self.frame <= self.init_time or status == STATUS_GOOD and (self.max_psr == None or psr < self.max_psr):
            self.filter.addTraining(tile,target,rate=self.update_rate)
                
            # Recenter the affine
            target = affine.invertPoint(target)
            dx = target.X() - self.rect.center().X()
            dy = target.Y() - self.rect.center().Y()
            if self.update_location:
                self.rect.x = self.rect.x + dx
                self.rect.y = self.rect.y + dy
            #self.center = target
                                
        stop = time.time()
        self.update_time = stop - start
           
                
    def asPoint(self):
        '''
        Return the center point of the track.
        
        @return: the track center point.
        @rtype: pv.Point
        '''
        return self.rect.center()
    
    def asRect(self):
        '''
        Return the current location of the track as a rectangle.
        
        @return: The tracking rectangle.
        @rtype: pv.Rect
        '''
        return self.rect
        
    def updateTime(self):
        '''
        Returns the update time for the last frame processed.
        
        @return: the update time in seconds for the last frame processed.
        @rtype:  float
        '''
        return self.update_time
    
    def filterAsImage(self):
        '''
        Returns the filter as an image that can be displayed.
        
        @return: The filter as an image.
        @rtype:  pv.Image
        '''
        return self.filter.asImage()

    def inputAsImage(self):
        '''
        Returns the correlation input as an image for the last frame processed.
        
        @return: the resampled input image.
        @rtype:  pv.Image
        '''
        return self.input
        
    def correlationAsImage(self):
        '''
        Returns the correlation output for the last frame processed.
        
        @return: The correlation output as an image.
        @rtype:  pv.Image
        '''
        return pv.Image(self.corr*(self.corr > 0.0))
    
    
    def psr(self):
        '''
        Compute the B{peak-to-sidelobe ratio} (PSR) of the correlation output.  This is 
        a good measure of quality. PSRs typically range from 4.0 to 70.0.  A 
        when the PSR drops to 7.0-9.0 typically indicates that the tracker is 
        having difficulty due to occlusion or fast appearance changes. It is 
        rare to see a tracking failure for a PSR above 10.0.  Note that the 
        quality method remaps PSR to a more intuitive range of [0.0,1.0].
        
        @return: The Peak-to-Sidelobe Ratio 
        @rtype:  float
        '''
        if self.frame < 1:
            return 0.0
        
        if self.psr_cache != None:
            return self.psr_cache
                
        _,cols = self.corr.shape

        # Find the peak
        i = self.corr.argmax()
        x,y = i/cols, i%cols
        pk = self.corr[x,y]
        
        # Mask out the sidelobe
        mask = np.ones(self.corr.shape,dtype=np.bool)
        mask[x-5:x+6,y-5:y+6] = False
        corr = self.corr.flatten()
        mask = mask.flatten()
        sidelobe = corr[mask]
        
        # compute the psr
        mn = sidelobe.mean()
        sd = sidelobe.std()
        self.psr_cache = (pk-mn)/sd
        return self.psr_cache
    

    def annotateFrame(self,frame,color="red",show_images=True):
        '''
        Annotates the image with tracking information including the frame 
        number, psr, and update time. It also displays the input, filter, 
        and output. This method requires quite a bit of time.
        
        @param frame: The frame to annotate this should be the frame that was just passed to the L{update} method.
        @type frame:  pv.Image
        '''        
        
        # Add some text data fto the frame
        frame.annotateLabel(pv.Point(10,10),"FRAME:       %4d"%self.frame)
        frame.annotateLabel(pv.Point(10,20),"PSR:         %6.2f"%self.psr())
        frame.annotateLabel(pv.Point(10,30),"UPDATE TIME: %6.2f ms"%(1000.0*self.updateTime()))
        
        # Show the new estimated center of the track
        estimate = self.asPoint()
        if estimate != None:
            frame.annotateCircle(estimate,5,color=color,fill=color)
        
        # Show the previous tracking location as a gray rectangle
        rect = self.prevRect
        if rect != None:
            w,h = self.tile_size
            tl = pv.Point(rect.x,rect.y)
            tr = pv.Point(rect.x+rect.w,rect.y)
            br = pv.Point(rect.x+rect.w,rect.y+rect.h)
            bl = pv.Point(rect.x,rect.y+rect.h)
            frame.annotateLine(tl,tr,color='gray',width=1)
            frame.annotateLine(tr,br,color='gray',width=1)
            frame.annotateLine(br,bl,color='gray',width=1)
            frame.annotateLine(bl,tl,color='gray',width=1)
    
        # Draw a rectangle for the new location of the track.
        rect = self.rect
        if rect != None:
            w,h = self.tile_size
            tl = pv.Point(rect.x,rect.y)
            tr = pv.Point(rect.x+rect.w,rect.y)
            br = pv.Point(rect.x+rect.w,rect.y+rect.h)
            bl = pv.Point(rect.x,rect.y+rect.h)
            frame.annotateLine(tl,tr,color=color,width=3)
            frame.annotateLine(tr,br,color=color,width=3)
            frame.annotateLine(br,bl,color=color,width=3)
            frame.annotateLine(bl,tl,color=color,width=3)
                        
            if self.getStatus() == STATUS_DROPPED:
                frame.annotateLine(tl,br,color=color,width=3)
                frame.annotateLine(bl,tr,color=color,width=3)
                
        # Show images of the input, filter, and output
        if  show_images:
            pil = frame.asAnnotated()
            x = 10
            y = frame.size[1] - 10
            
            # The input images
            input = self.inputAsImage()
            w,h = input.size
            pil.paste(input.asAnnotated(),(x,y-h))
            frame.annotateRect(pv.Rect(x,y-h,w,h),color=color)
            frame.annotateLabel(pv.Point(x,y-h-10),"INPUT")
            x = x + w + 10
            
            # The correlation filter
            filter = self.filterAsImage()
            w,h = filter.size
            pil.paste(filter.asAnnotated(),(x,y-h))
            frame.annotateRect(pv.Rect(x,y-h,w,h),color=color)
            frame.annotateLabel(pv.Point(x,y-h-10),"FILTER")
            x = x + w + 10
            
            # The correlation output
            corr = self.correlationAsImage()
            w,h = corr.size
            pil.paste(corr.asAnnotated(),(x,y-h))
            frame.annotateRect(pv.Rect(x,y-h,w,h),color=color)
            frame.annotateLabel(pv.Point(x,y-h-10),"OUTPUT")
            x = x + w + 10
            

class _TrackingFilter(common.CorrelationFilter):
    '''
    This private class implements a fast and simple filter for  a fixed image 
    size.  This class handles all of the filtering while L{MOSSETrack}
    handles the cropping and maintaining the status of the track.  Typically
    this class should only be created and accessed via the MOSSETrack public 
    interface.
    '''
    
    def __init__(self, tile_size,type="MOSSE", sigma=3.0, reg=0.1, **kwargs):
        
        # initialize filter parameters
        self.type = type
        self.sigma = sigma
        self.tile_size=tile_size
        self.reg = reg

        # Initialize translation filters which are used to shift the target
        # output image quickly in the Fourier domain and speed up the tracking
        # process.
        ocof.translationFilter.initSize(tile_size)

        # H is used for an ASEF Filter
        self.H = None
        
        self.n_training = 0
        
        w,h = tile_size
        cx = int(tile_size[0]/2)
        cy = int(tile_size[1]/2)
        
        # Select the filter type
        if self.type in ['ASEF','MOSSE']:
            peak = createPointTarget(cx,cy,tile_size,self.sigma)
        elif self.type in ['MEAN','UMACE']:
            peak = createDeltaTarget(cx,cy,tile_size)
        else:
            raise NotImplementedError("Unknown Filter Type: %s"%self.type)
         
        # Used to help compute the center of mass
        self.x_array = np.fft.fftshift(np.arange(w).reshape((w,1)) - cx)
        self.y_array = np.fft.fftshift(np.arange(h).reshape((1,h)) - cy)

        # This recenters the peak        
        self.peak = ocof.translationFilter(-cx,-cy,tile_size) * np.fft.fft2(peak)
                
        # Call the super class constructor
        common.CorrelationFilter.__init__(self,tile_size,**kwargs)
    
        
    def correlate(self,im,ilog=None):
        '''
        @return: the correlation of the image with the filter.
        '''
        assert im.size == self.tile_size
        
        # Implement correlation for each filters type
        if self.type in ['ASEF']:
            fftfilter = self.H            
        elif self.type in ['MOSSE','UMACE']:
            fftfilter = self.N.conj()/self.D
        elif self.type in ['MEAN']:
            print "MEAN"
            fftfilter = self.N.conj()
        else:
            raise NotImplementedError("Unknown Filter Type: %s"%self.type)
            
        # Preprocess the input image
        mat = self.preprocess(im)
        
        # Perform the fft correlation
        fftmat = np.fft.fft2(mat)
        fftcor = fftfilter.conj()*fftmat
        cor = np.fft.ifft2(fftcor)
        
        # Return the real portion of the result.
        return cor.real

    def asSpatial(self):
        '''
        @return: the filter in the spatial domain as a numpy array.
        @rtype: np.array
        '''
        if self.type in ['ASEF']:
            fftfilter = self.H            
        elif self.type in ['MOSSE','UMACE']:
            fftfilter = self.N.conj()/self.D
        elif self.type in ['MEAN']:
            fftfilter = self.N.conj()
        else:
            raise NotImplementedError("Unknown Filter Type: %s"%self.type)
        tmp = np.fft.ifft2(fftfilter)
        return tmp.real
    
    def asImage(self):
        '''
        @return: the filter in the spatial domain as a numpy array.
        @rtype: np.array
        '''
        tmp = self.asSpatial()
        tmp = np.fft.fftshift(tmp)
        return pv.Image(tmp)

    def addTraining(self, im, pt, rate = None, vals=1.0, ilog=None):
        ''' add a training point  '''            
        assert im.size == self.tile_size
                
        data = self.preprocess(im)                
        
        # Make sure number of points is > 0 and transforme the points to the new position
        T = ocof.translationFilter(int(pt.X()),int(pt.Y()),self.tile_size) * self.peak
            
        # solve for the filter
        X = np.fft.fft2(data)
        N = T*X.conj()
        H = ((T*X.conj())/((X*X.conj()).real + self.reg)).conj()
        D = X*X.conj()+self.reg
                    
        self.n_training += 1

        # save the state or compute the update if rate != None
        if self.H is None:
            self.H = H
            self.N = N
            self.D = D
        elif rate == None:
            w1 = (self.n_training-1.0)/self.n_training
            w2 = 1.0/self.n_training
            self.H = w1*self.H + w2*H
            self.N = w1*self.N + w2*N
            self.D = w1*self.D + w2*D
        elif rate < 1e-7:
            pass
        else:
            assert rate >= 0.0
            assert rate <= 1.0
            w1 = 1.0-rate
            w2 = rate
            self.H = w1*self.H + w2*H
            self.N = w1*self.N + w2*N
            self.D = w1*self.D + w2*D
                    
         
def createPointTarget(x,y,size,sigma):
    '''
    This function creates a target array that is used train ASEF and MOSSE 
    filters. The peaks are shaped like two dimensional Gaussians of radius
    sigma.
    
    @param x: the x location of the peak.
    @type x: float
    @param y: the y location of the peak.
    @type y: float
    @param size: (width,height).
    @type size: (int,int)
    @param sigma: Gaussian radius.
    @type sigma: float
    @return: A numpy array that is mostly zero except for the peak.
    @rtype: np.array
    '''
    x = np.arange(size[0])-x
    y = np.arange(size[1])-y
    scale = 1.0/(sigma*sigma)
    target = np.exp(-scale*x*x).reshape(size[0],1)*np.exp(-scale*y*y).reshape(1,size[1])
    return target

def createDeltaTarget(x,y,size):
    '''
    This function creates a target array that is used train ASEF and MOSSE 
    filters. The peaks are shaped like Delta function with a value of 1.0
    at the center of the target and 0.0 elsewhere in the array.  (Used to
    approximate MEAN or UMACE filters.)
    
    @param x: the x location of the peak.
    @type x: float
    @param y: the y location of the peak.
    @type y: float
    @param size: (width,height).
    @type size: (int,int)
    @return: A numpy array that is mostly zero except for the peak.
    @rtype: np.array
    '''
    x = int(round(x))
    y = int(round(y))
    target = np.zeros(size,dtype=np.float64)
    target[x,y] = 1.0
    return target





