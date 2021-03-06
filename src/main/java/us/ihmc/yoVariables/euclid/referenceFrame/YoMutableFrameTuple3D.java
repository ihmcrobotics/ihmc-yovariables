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
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FrameTuple3DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoMutableFrameTuple3D extends YoTuple3D implements FrameTuple3DBasics, YoMutableFrameObject
{
   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components to zero and its reference
    * frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry);
      setToZero(referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    * @param x              the initial value for the x-component.
    * @param y              the initial value for the y-component.
    * @param z              the initial value for the z-component.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, x, y, z);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    * @param tupleArray     the array containing this tuple's components. Not modified.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this tuple.
    * @param registry        the registry to register child variables to.
    * @param tuple2DReadOnly the tuple used to initializes this tuple's components. Not modified.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this tuple.
    * @param registry        the registry to register child variables to.
    * @param tuple3DReadOnly the tuple used to initializes this tuple's components. Not modified.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple2DReadOnly the tuple used to initializes this tuple's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple3DReadOnly the tuple used to initializes this tuple's components and reference
    *                             frame. Not modified.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      this(namePrefix, nameSuffix, registry);
      setIncludingFrame(frameTuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D}, initializes its components to zero and its reference
    * frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFrameTuple3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      frameId = new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame", nameSuffix), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();
      setReferenceFrame(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new {@code YoMutableFrameTuple3D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param x             the variable to use for the x-coordinate.
    * @param y             the variable to use for the y-coordinate.
    * @param z             the variable to use for the z-coordinate.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFrameTuple3D(YoDouble x, YoDouble y, YoDouble z, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y, z);
      this.frameId = frameIndex;
      this.frameIndexMap = frameIndexMap;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple3DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple3DReadOnly)
         return equals((FrameTuple3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
   }
}
