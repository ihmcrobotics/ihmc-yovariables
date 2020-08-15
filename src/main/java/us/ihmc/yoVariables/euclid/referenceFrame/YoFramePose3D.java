/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code FixedFramePose3DBasics} implementation which position and orientation backed with
 * {@code YoVariable}s.
 */
public class YoFramePose3D implements FixedFramePose3DBasics
{
   private final YoFramePoint3D position;
   private final YoFrameQuaternion orientation;

   /**
    * Creates a new {@code YoFramePose3D} using the given {@code position} and {@code orientation}.
    *
    * @param position    the {@code YoFramePoint3D} to use internally for the position.
    * @param orientation the {@code YoFrameQuaternion} to use internally for the orientation.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   public YoFramePose3D(YoFramePoint3D position, YoFrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      position.getReferenceFrame();
      this.position = position;
      this.orientation = orientation;
   }

   /**
    * Creates a new {@code YoFramePose3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this pose.
    * @param registry       the registry to register child variables to.
    */
   public YoFramePose3D(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFramePose3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this pose.
    * @param registry       the registry to register child variables to.
    */
   public YoFramePose3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      position = new YoFramePoint3D(namePrefix, nameSuffix, referenceFrame, registry);
      orientation = new YoFrameQuaternion(namePrefix, nameSuffix, referenceFrame, registry);
   }

   /**
    * Sets this pose to represent the same geometry as the given {@code yoFramePose}.
    *
    * @param yoFramePose the pose used to set this. Not modified.
    */
   public void set(YoFramePoseUsingYawPitchRoll yoFramePose)
   {
      set(yoFramePose.getPosition(), yoFramePose.getOrientation());
   }

   /**
    * Sets this frame pose to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePose3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame pose to set this to. Not modified.
    */
   @Override
   public void setMatchingFrame(FramePose3DReadOnly other)
   {
      position.setMatchingFrame(other.getPosition());
      orientation.setMatchingFrame(other.getOrientation());
   }

   /**
    * Sets this frame pose to {@code other}.
    * <p>
    * If the arguments are expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameTuple3DReadOnly, FrameOrientation3DReadOnly)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * to {@code position} and {@code orientation} once transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position    the frame point used to set this position to. Not modified.
    * @param orientation the frame quaternion to set this orientation to. Not modified.
    */
   public void setMatchingFrame(FrameTuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      this.position.setMatchingFrame(position);
      this.orientation.setMatchingFrame(orientation);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   public YoFramePoint3D getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Gets the internal reference to the x-coordinate used for the position of this pose.
    *
    * @return the position x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   /**
    * Gets the internal reference to the y-coordinate used for the position of this pose.
    *
    * @return the position y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   /**
    * Gets the internal reference to the z-coordinate used for the position of this pose.
    *
    * @return the position z-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   /**
    * Gets the internal reference to the s-component used for the orientation of this pose.
    *
    * @return the position s-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQs()
   {
      return orientation.getYoQs();
   }

   /**
    * Gets the internal reference to the x-component used for the orientation of this pose.
    *
    * @return the position x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQx()
   {
      return orientation.getYoQx();
   }

   /**
    * Gets the internal reference to the y-component used for the orientation of this pose.
    *
    * @return the position y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQy()
   {
      return orientation.getYoQy();
   }

   /**
    * Gets the internal reference to the z-component used for the orientation of this pose.
    *
    * @return the position z-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQz()
   {
      return orientation.getYoQz();
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this pose components change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public void attachVariableChangedListener(YoVariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      orientation.attachVariableChangedListener(variableChangedListener);
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFramePose3D duplicate(YoRegistry newRegistry)
   {
      return new YoFramePose3D(position.duplicate(newRegistry), orientation.duplicate(newRegistry));
   }

   /**
    * Provides a {@code String} representation of this pose as follows:<br>
    * Pose 3D: position = (x, y, z), orientation = (x, y, z, s)-worldFrame
    *
    * @return the {@code String} representing this pose.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFramePose3DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePose3DReadOnly)
         return equals((FramePose3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getOrientation());
   }
}
