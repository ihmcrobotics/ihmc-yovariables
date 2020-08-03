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

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tools.YawPitchRollTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Defines a 3D pose in a fixed-frame as {@code YoFramePose3D} but use the Euler angles to represent
 * the orientation part.
 *
 * @see YoFramePose3D
 * @see YoFrameYawPitchRoll
 */
public class YoFramePoseUsingYawPitchRoll implements FramePose3DReadOnly, Clearable, Transformable
{
   private final YoFramePoint3D position;
   private final YoFrameYawPitchRoll yawPitchRoll;

   private final FrameQuaternion frameQuaternion = new FrameQuaternion();

   /**
    * Creates a new {@code YoFramePoseUsingYawPitchRoll} using the given {@code position} and
    * {@code orientation}.
    *
    * @param position    the {@code YoFramePoint3D} to use internally for the position.
    * @param orientation the {@code YoFrameYawPitchRoll} to use internally for the orientation.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *                                         expressed in the same reference frame.
    */
   public YoFramePoseUsingYawPitchRoll(YoFramePoint3D position, YoFrameYawPitchRoll orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position = position;
      yawPitchRoll = orientation;
   }

   /**
    * Creates a new {@code YoFramePoseUsingYawPitchRoll}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this pose.
    * @param registry       the registry to register child variables to.
    */
   public YoFramePoseUsingYawPitchRoll(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFramePoseUsingYawPitchRoll}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this pose.
    * @param registry       the registry to register child variables to.
    */
   public YoFramePoseUsingYawPitchRoll(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      position = new YoFramePoint3D(namePrefix, nameSuffix, referenceFrame, registry);
      yawPitchRoll = new YoFrameYawPitchRoll(namePrefix, nameSuffix, referenceFrame, registry);
   }

   /** {@inheritDoc} */
   @Override
   public YoFramePoint3D getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      frameQuaternion.setIncludingFrame(yawPitchRoll);
      return frameQuaternion;
   }

   /**
    * Returns the reference to the internal orientation part of this pose.
    * 
    * @return the orientation part of this pose.
    */
   public YoFrameYawPitchRoll getYawPitchRoll()
   {
      return yawPitchRoll;
   }

   /**
    * Sets the position coordinates.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    */
   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   public void setPosition(Tuple3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the x and y coordinates from the given tuple 2D, the z coordinate remains unchanged.
    * <p>
    * The z component remains unchanged.
    * </p>
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    */
   public void setPosition(Tuple2DReadOnly position2D)
   {
      position.set(position2D);
   }

   /**
    * Sets the position from the given tuple 2D and z coordinate.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    * @param z          the new z value for this pose's position z-coordinate.
    */
   public void setPosition(Tuple2DReadOnly position2D, double z)
   {
      position.set(position2D, z);
   }

   /**
    * Sets all the components of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param x     the x-coordinate of the position.
    * @param y     the y-coordinate of the position.
    * @param z     the z-coordinate of the position.
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public void set(double x, double y, double z, double yaw, double pitch, double roll)
   {
      setPosition(x, y, z);
      setOrientationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to the given {@code pose2DReadOnly}.
    *
    * @param pose2DReadOnly the pose 2D used to set this pose 3D. Not modified.
    */
   public void set(Pose2DReadOnly pose2DReadOnly)
   {
      setPosition(pose2DReadOnly.getPosition(), 0.0);
      setOrientation(pose2DReadOnly.getOrientation());
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    *
    * @param other the other pose 3D. Not modified.
    */
   public void set(Pose3DReadOnly other)
   {
      setPosition(other.getPosition());
      setOrientation(other.getOrientation());
   }

   /**
    * Sets the orientation part of this pose 3D with the given orientation 2D.
    *
    * @param orientation the orientation 2D used to set this pose's orientation. Not modified.
    */
   public void setOrientation(Orientation2DReadOnly orientation)
   {
      yawPitchRoll.set(orientation.getYaw(), 0.0, 0.0);
   }

   /**
    * Sets the orientation part of this pose 3D with the given orientation.
    *
    * @param orientation the orientation used to set this pose's orientation. Not modified.
    */
   public void setOrientation(Orientation3DReadOnly orientation)
   {
      yawPitchRoll.set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      yawPitchRoll.setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to match the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform use to set this pose 3D. Not modified.
    */
   public void set(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      setPosition(rigidBodyTransform.getTranslation());
      setOrientation(rigidBodyTransform.getRotation());
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    */
   public void set(Tuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      setOrientation(orientation);
      setPosition(position);
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return yawPitchRoll.containsNaN() || position.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      yawPitchRoll.setToNaN();
      position.setToNaN();
   }

   /**
    * Sets the position to (0, 0) and the orientation to the neutral quaternion, i.e. zero rotation.
    */
   @Override
   public void setToZero()
   {
      yawPitchRoll.setToZero();
      position.setToZero();
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    */
   public void interpolate(Pose3DReadOnly other, double alpha)
   {
      position.interpolate(other.getPosition(), alpha);
      frameQuaternion.setReferenceFrame(getReferenceFrame());
      frameQuaternion.interpolate(other.getOrientation(), alpha);
      yawPitchRoll.set(frameQuaternion);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    */
   public void interpolate(Pose3DReadOnly pose1, Pose3DReadOnly pose2, double alpha)
   {
      position.interpolate(pose1.getPosition(), pose2.getPosition(), alpha);
      frameQuaternion.setReferenceFrame(getReferenceFrame());
      frameQuaternion.interpolate(pose1.getOrientation(), pose2.getOrientation(), alpha);
      yawPitchRoll.set(frameQuaternion);
   }

   /**
    * Adds the translation (x, y, z) to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the translation is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   public void prependTranslation(double x, double y, double z)
   {
      position.add(x, y, z);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   public void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   public void prependRotation(Orientation3DReadOnly rotation)
   {
      rotation.transform(position);
      rotation.transform(yawPitchRoll);
   }

   /**
    * Prepends a rotation about the z-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, position, position);
      yawPitchRoll.prependYawRotation(yaw);
   }

   /**
    * Prepends a rotation about the y-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, position, position);
      yawPitchRoll.prependPitchRotation(pitch);
   }

   /**
    * Prepends a rotation about the x-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, position, position);
      yawPitchRoll.prependRollRotation(roll);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    *
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   public void prependTransform(RigidBodyTransformReadOnly transform)
   {
      applyTransform(transform);
   }

   /**
    * Rotates, then adds the translation (x, y, z) to this pose 3D.
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   public void appendTranslation(double x, double y, double z)
   {
      double thisX = getX();
      double thisY = getY();
      double thisZ = getZ();

      setPosition(x, y, z);
      yawPitchRoll.transform(position);
      position.add(thisX, thisY, thisZ);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   public void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Appends the given orientation to this pose 3D.
    * <p>
    * Only the orientation part of this pose is affected by this operation, for more details see
    * {@link Orientation3DBasics#append(Orientation3DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to append to this pose 3D. Not modified.
    */
   public void appendRotation(Orientation3DReadOnly orientation)
   {
      yawPitchRoll.append(orientation);
   }

   /**
    * Appends a rotation about the z-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / cos(yaw) -sin(yaw) 0 \
    * this.orientation = this.orientation * | sin(yaw)  cos(yaw) 0 |
    *                                       \    0         0     1 /
    * </pre>
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      yawPitchRoll.appendYawRotation(yaw);
   }

   /**
    * Appends a rotation about the y-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       /  cos(pitch) 0 sin(pitch) \
    * this.orientation = this.orientation * |      0      1     0      |
    *                                       \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      yawPitchRoll.appendPitchRotation(pitch);
   }

   /**
    * Appends a rotation about the x-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / 1     0          0     \
    * this.orientation = this.orientation * | 0 cos(roll) -sin(roll) |
    *                                       \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      yawPitchRoll.appendRollRotation(roll);
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    *
    * @param transform the rigid-body transform to append to this pose 3D. Not modified.
    */
   public void appendTransform(RigidBodyTransformReadOnly transform)
   {
      YawPitchRollTools.addTransform(yawPitchRoll, transform.getTranslation(), position);
      yawPitchRoll.append(transform.getRotation());
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the given {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
      transform.transform(yawPitchRoll);
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the inverse of the given
    * {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(position);
      transform.inverseTransform(yawPitchRoll);
   }

   /**
    * Sets this pose 3D to represent the pose of the given {@code referenceFrame} expressed in
    * {@code this.getReferenceFrame()}.
    *
    * @param referenceFrame the reference frame of interest.
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the position from the given frame tuple 2D.
    * <p>
    * The z component remains unchanged.
    * </p>
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   public void setPosition(FrameTuple2DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      setPosition((Tuple2DReadOnly) position);
   }

   /**
    * Sets the position from the given frame tuple 2D and the given {@code z} coordinate.
    *
    * @param position the tuple with the x and y position coordinates. Not modified.
    * @param z        the new z-coordinate.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   public void setPosition(FrameTuple2DReadOnly position, double z)
   {
      checkReferenceFrameMatch(position);
      setPosition((Tuple2DReadOnly) position, z);
   }

   /**
    * Sets the position from the given frame tuple 3D.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   public void setPosition(FrameTuple3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      setPosition((Tuple3DReadOnly) position);
   }

   /**
    * Sets the orientation from the given frame orientation 2D.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   public void setOrientation(FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      setOrientation((Orientation2DReadOnly) orientation);
   }

   /**
    * Sets the orientation from the given frame orientation.
    *
    * @param orientation the orientation to set the orientation part of this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   public void setOrientation(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      setOrientation((Orientation3DReadOnly) orientation);
   }

   /**
    * Sets the pose to represent the same pose as the given {@code pose2DReadOnly} that is expressed in
    * the given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the given {@code pose2DReadOnly} is expressed
    *                       in.
    * @param pose2DReadOnly the pose 2D used to set the pose of this frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   public void set(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose2DReadOnly);
   }

   /**
    * Sets the pose from the given {@code pose3DReadOnly} that is expressed in the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the given {@code pose3DReadOnly} is expressed
    *                       in.
    * @param pose3DReadOnly the pose 3D used to set the pose of this frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   public void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose3DReadOnly);
   }

   /**
    * Sets this frame pose 3D to the represent the same pose as the given {@code framePose2DReadOnly}.
    *
    * @param framePose2DReadOnly the other frame pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code framePose2DReadOnly} are not
    *                                         expressed in the same reference frame.
    */
   public void set(FramePose2DReadOnly framePose2DReadOnly)
   {
      checkReferenceFrameMatch(framePose2DReadOnly);
      set((Pose2DReadOnly) framePose2DReadOnly);
   }

   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   public void set(FramePose3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((Pose3DReadOnly) other);
   }

   /**
    * Sets this pose 3D to be the same as the given one expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePose3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame pose 3D to set this to. Not modified.
    */
   public void setMatchingFrame(FramePose3DReadOnly other)
   {
      set((Pose3DReadOnly) other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   public void set(FrameTuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      set((Tuple3DReadOnly) position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   public void set(Tuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      set(position, (Orientation3DReadOnly) orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code position}, and
    *                                         {@code orientation} are not expressed in the same
    *                                         reference frame.
    */
   public void set(FrameTuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      set((Tuple3DReadOnly) position, (Orientation3DReadOnly) orientation);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   public void prependTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      prependTranslation((Tuple3DReadOnly) translation);
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not expressed in
    *                                         the same reference frame.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   public void prependRotation(FrameOrientation3DReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      prependRotation((Orientation3DReadOnly) rotation);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   public void appendTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      appendTranslation((Tuple3DReadOnly) translation);
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    *
    * @param rotation the rotation to append to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not expressed in
    *                                         the same reference frame.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   public void appendRotation(FrameOrientation3DReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      appendRotation((Orientation3DReadOnly) rotation);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   public void interpolate(FramePose3DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      interpolate((Pose3DReadOnly) other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not expressed in
    *                                         the same reference frame.
    */
   public void interpolate(FramePose3DReadOnly pose1, Pose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      interpolate((Pose3DReadOnly) pose1, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not expressed in
    *                                         the same reference frame.
    */
   public void interpolate(Pose3DReadOnly pose1, FramePose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose2);
      interpolate(pose1, (Pose3DReadOnly) pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1}, and {@code pose2} are not
    *                                         expressed in the same reference frame.
    */
   public void interpolate(FramePose3DReadOnly pose1, FramePose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      checkReferenceFrameMatch(pose2);
      interpolate((Pose3DReadOnly) pose1, (Pose3DReadOnly) pose2, alpha);
   }

   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   public void set(YoFramePoseUsingYawPitchRoll other)
   {
      set(other.position, other.yawPitchRoll);
   }

   /**
    * Sets this pose 3D to be the same as the given one expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(YoFramePoseUsingYawPitchRoll)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame pose 3D to set this to. Not modified.
    */
   public void setMatchingFrame(YoFramePoseUsingYawPitchRoll other)
   {
      setMatchingFrame(other.position, other.yawPitchRoll);
   }

   /**
    * Sets this pose 3D to the given {@code position} and {@code orientation} expressed in the
    * reference frame of this.
    * <p>
    * If the arguments are expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameTuple3DReadOnly, FrameOrientation3DReadOnly)}.
    * </p>
    * <p>
    * If the arguments are expressed in a different frame than {@code this}, then {@code this} is set
    * and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    */
   public void setMatchingFrame(FrameTuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      this.position.setMatchingFrame(position);
      yawPitchRoll.setMatchingFrame(orientation);
   }

   /**
    * Adds the components of {@code other} to this components.
    * 
    * @param other the other pose to add the components to {@code this}. Not modified.
    */
   public void add(YoFramePoseUsingYawPitchRoll other)
   {
      position.add(other.position);
      yawPitchRoll.add(other.yawPitchRoll);
   }

   /**
    * Sets the yaw angle of the orientation part.
    *
    * @param yaw the new yaw angle.
    */
   public void setYaw(double yaw)
   {
      yawPitchRoll.setYaw(yaw);
   }

   /**
    * Sets the pitch angle of the orientation part.
    *
    * @param pitch the new pitch angle.
    */
   public void setPitch(double pitch)
   {
      yawPitchRoll.setPitch(pitch);
   }

   /**
    * Sets the roll angle of the orientation part.
    *
    * @param roll the new roll angle.
    */
   public void setRoll(double roll)
   {
      yawPitchRoll.setRoll(roll);
   }

   /**
    * Sets the x-coordinate of the position part.
    * 
    * @param x the position along the x-axis.
    */
   public void setX(double x)
   {
      position.setX(x);
   }

   /**
    * Sets the y-coordinate of the position part.
    * 
    * @param y the position along the y-axis.
    */
   public void setY(double y)
   {
      position.setY(y);
   }

   /**
    * Sets the z-coordinate of the position part.
    * 
    * @param z the position along the z-axis.
    */
   public void setZ(double z)
   {
      position.setZ(z);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return position.getReferenceFrame();
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this pose components change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public void attachVariableChangedListener(YoVariableChangedListener variableChangedListener)
   {
      position.attachVariableChangedListener(variableChangedListener);
      yawPitchRoll.attachVariableChangedListener(variableChangedListener);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return position.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return position.getY();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return position.getZ();
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return yawPitchRoll.getRoll();
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return yawPitchRoll.getPitch();
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yawPitchRoll.getYaw();
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
    * Gets the internal reference to the yaw angle used for the orientation of this pose.
    *
    * @return the angle around the z-axis as {@code YoVariable}.
    */
   public YoDouble getYoYaw()
   {
      return yawPitchRoll.getYoYaw();
   }

   /**
    * Gets the internal reference to the pitch angle used for the orientation of this pose.
    *
    * @return the angle around the y-axis as {@code YoVariable}.
    */
   public YoDouble getYoPitch()
   {
      return yawPitchRoll.getYoPitch();
   }

   /**
    * Gets the internal reference to the roll angle used for the orientation of this pose.
    *
    * @return the angle around the x-axis as {@code YoVariable}.
    */
   public YoDouble getYoRoll()
   {
      return yawPitchRoll.getYoRoll();
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
   public YoFramePoseUsingYawPitchRoll duplicate(YoRegistry newRegistry)
   {
      return new YoFramePoseUsingYawPitchRoll(position.duplicate(newRegistry), yawPitchRoll.duplicate(newRegistry));
   }

   /**
    * Tests if this pose is exactly equal to {@code other}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   public boolean equals(YoFramePoseUsingYawPitchRoll other)
   {
      if (other == null)
         return false;
      else if (other == this)
         return true;
      else
         return position.equals(other.position) && yawPitchRoll.equals(other.yawPitchRoll);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof YoFramePoseUsingYawPitchRoll)
         return equals((YoFramePoseUsingYawPitchRoll) object);
      else if (object instanceof FramePose3DReadOnly)
         return FramePose3DReadOnly.super.equals((FramePose3DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFramePose3DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getPosition(), getOrientation());
   }
}
