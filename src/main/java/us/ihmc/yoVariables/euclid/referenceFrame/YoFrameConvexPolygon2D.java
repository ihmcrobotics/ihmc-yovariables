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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Convexity;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBoundingBox2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * {@code FixedFrameConvexPolygon2DBasics} implementation which components vertices are backed with
 * {@code YoFramePoint2D}s.
 */
public class YoFrameConvexPolygon2D implements FixedFrameConvexPolygon2DBasics
{
   private final List<YoFramePoint2D> yoVertexBuffer = new ArrayList<>();
   private final List<FixedFramePoint2DBasics> vertexBufferView = Collections.unmodifiableList(yoVertexBuffer);
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in clockwise
    * or counter-clockwise ordered.
    */
   private final boolean clockwiseOrdered = true;
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();
   private final YoInteger numberOfVertices;
   /** The reference frame in which this polygon is currently expressed. */
   private final ReferenceFrame referenceFrame;

   /**
    * The smallest axis-aligned bounding box that contains all this polygon's vertices.
    * <p>
    * It is updated in the method {@link #updateBoundingBox()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final FixedFrameBoundingBox2DBasics boundingBox = EuclidFrameFactories.newFixedFrameBoundingBox2DBasics(this);
   /**
    * The centroid of this polygon which is located at the center of mass of this polygon when
    * considered as a physical object with constant thickness and density.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final FixedFramePoint2DBasics centroid = EuclidFrameFactories.newFixedFramePoint2DBasics(this);
   /**
    * The area of this convex polygon.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    * <p>
    * When a polygon is empty, i.e. has no vertices, the area is equal to {@link Double#NaN}.
    * </p>
    */
   private double area;
   /**
    * This field is used to know whether the method {@link #update()} has been called since the last
    * time the vertices of this polygon have been modified.
    * <p>
    * Most operations with a polygon require the polygon to be up-to-date.
    * </p>
    */
   private boolean isUpToDate = false;
   private boolean boundingBoxDirty = true;
   private boolean areaCentroidDirty = true;
   /** Vertex to store intermediate results to allow garbage free operations. */
   private final Point3D vertex3D = new Point3D();
   /** For the update method, this is to prevent changing the order of the yoVertexBuffer. */
   private final List<Point2D> tempVertexBuffer = new ArrayList<>();

   /**
    * Creates a new empty polygon.
    *
    * @param namePrefix          a unique name string to use as the prefix for child variable names.
    * @param referenceFrame      the reference frame in which this polygon will <b>always</b> be
    *                            expressed.
    * @param maxNumberOfVertices refers to the number of {@code YoVariable}s to be created. This
    *                            polygon cannot grow bigger than this number.
    * @param registry            the registry to register child variables to.
    */
   public YoFrameConvexPolygon2D(String namePrefix, ReferenceFrame referenceFrame, int maxNumberOfVertices, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, maxNumberOfVertices, registry);
   }

   /**
    * Creates a new empty polygon.
    *
    * @param namePrefix          a unique name string to use as the prefix for child variable names.
    * @param nameSuffix          a string to use as the suffix for child variable names.
    * @param referenceFrame      the reference frame in which this polygon will <b>always</b> be
    *                            expressed.
    * @param maxNumberOfVertices refers to the number of {@code YoVariable}s to be created. This
    *                            polygon cannot grow bigger than this number.
    * @param registry            the registry to register child variables to.
    */
   public YoFrameConvexPolygon2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, int maxNumberOfVertices, YoRegistry registry)
   {
      numberOfVertices = new YoInteger(namePrefix + "NumVertices" + nameSuffix, registry);

      this.referenceFrame = referenceFrame;

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         YoFramePoint2D point = new YoFramePoint2D(namePrefix + "_" + i + "_", nameSuffix, referenceFrame, registry);
         yoVertexBuffer.add(point);
         tempVertexBuffer.add(new Point2D());
      }
   }

   /**
    * Creates a new empty polygon.
    *
    * @param yoVertexBuffer     the buffer of vertices backed by {@code YoFramePoint2D}s to be used by
    *                           this polygon.
    * @param yoNumberOfVertices the {@code YoVariable} to be used by this polygon.
    * @param referenceFrame     the reference frame in which this polygon will <b>always</b> be
    *                           expressed.
    */
   public YoFrameConvexPolygon2D(List<YoFramePoint2D> yoVertexBuffer, YoInteger yoNumberOfVertices, ReferenceFrame referenceFrame)
   {
      numberOfVertices = yoNumberOfVertices;

      this.referenceFrame = referenceFrame;

      for (YoFramePoint2D vertex : yoVertexBuffer)
      {
         this.yoVertexBuffer.add(vertex);
         tempVertexBuffer.add(new Point2D());
      }
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return yoVertexBuffer.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
   }

   /** {@inheritDoc} */
   @Override
   public void clear()
   {
      numberOfVertices.set(0);
      area = Double.NaN;
      centroid.setToNaN();
      boundingBox.setToNaN();
      isUpToDate = false;
      boundingBoxDirty = true;
      areaCentroidDirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
      boundingBoxDirty = false;
      areaCentroidDirty = false;
   }

   @Override
   public void set(Vertex2DSupplier vertex2DSupplier)
   {
      if (vertex2DSupplier instanceof ConvexPolygon2DReadOnly)
      {
         ConvexPolygon2DReadOnly other = (ConvexPolygon2DReadOnly) vertex2DSupplier;

         if (clockwiseOrdered != other.isClockwiseOrdered())
         {
            // TODO For now relying on the expensive method to ensure consistent ordering.
            FixedFrameConvexPolygon2DBasics.super.set(vertex2DSupplier);
            return;
         }

         clear();
         numberOfVertices.set(other.getNumberOfVertices());

         for (int i = 0; i < numberOfVertices.getValue(); i++)
         {
            Point2DReadOnly otherVertex = other.getVertexUnsafe(i);
            if (i >= yoVertexBuffer.size())
               throw new RuntimeException("This polygon has reached its maximum number of vertices.");
            yoVertexBuffer.get(i).set(otherVertex);
         }

         if (other.isUpToDate())
         {
            isUpToDate = true;
            boundingBoxDirty = true;
            areaCentroidDirty = true;
         }
      }
      else
      {
         FixedFrameConvexPolygon2DBasics.super.set(vertex2DSupplier);
      }
   }

   public void set(YoFrameConvexPolygon2D other)
   {
      if (clockwiseOrdered != other.clockwiseOrdered)
      {
         // TODO For now relying on the expensive method to ensure consistent ordering.
         FixedFrameConvexPolygon2DBasics.super.set(other);
         return;
      }

      checkReferenceFrameMatch(other);

      numberOfVertices.set(other.numberOfVertices.getValue());

      for (int i = 0; i < other.numberOfVertices.getValue(); i++)
      {
         FixedFramePoint2DBasics otherVertex = other.yoVertexBuffer.get(i);
         if (i >= yoVertexBuffer.size())
            throw new RuntimeException("This polygon has reached its maximum number of vertices.");
         yoVertexBuffer.get(i).set(otherVertex);
      }
      boundingBox.set(other.boundingBox);
      centroid.set(other.centroid);
      area = other.area;
      isUpToDate = other.isUpToDate;
      boundingBoxDirty = other.boundingBoxDirty;
      areaCentroidDirty = other.areaCentroidDirty;
   }

   @Override
   public void set(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      if (frameVertex2DSupplier instanceof YoFrameConvexPolygon2D)
      {
         set((YoFrameConvexPolygon2D) frameVertex2DSupplier);
      }
      else if (frameVertex2DSupplier instanceof FrameConvexPolygon2DReadOnly)
      {
         FrameConvexPolygon2DReadOnly other = (FrameConvexPolygon2DReadOnly) frameVertex2DSupplier;

         if (clockwiseOrdered != other.isClockwiseOrdered())
         {
            // TODO For now relying on the expensive method to ensure consistent ordering.
            FixedFrameConvexPolygon2DBasics.super.set(frameVertex2DSupplier);
            return;
         }

         clear();
         numberOfVertices.set(other.getNumberOfVertices());

         for (int i = 0; i < numberOfVertices.getValue(); i++)
         {
            FramePoint2DReadOnly otherVertex = other.getVertexUnsafe(i);
            if (i >= yoVertexBuffer.size())
               throw new RuntimeException("This polygon has reached its maximum number of vertices.");
            yoVertexBuffer.get(i).set(otherVertex);
         }

         if (other.isUpToDate())
         {
            isUpToDate = true;
            boundingBoxDirty = true;
            areaCentroidDirty = true;
         }
      }
      else
      {
         FixedFrameConvexPolygon2DBasics.super.set(frameVertex2DSupplier);
      }
   }

   @Override
   public void setMatchingFrame(FrameVertex2DSupplier frameVertex2DSupplier, boolean checkIfTransformInXYPlane)
   {
      set((Vertex2DSupplier) frameVertex2DSupplier);

      if (frameVertex2DSupplier.getReferenceFrame() != referenceFrame)
      {
         frameVertex2DSupplier.getReferenceFrame().getTransformToDesiredFrame(transformToDesiredFrame, referenceFrame);
         applyTransform(transformToDesiredFrame, checkIfTransformInXYPlane);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point2DReadOnly vertex, boolean checkIfTransformInXYPlane)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         addVertex(vertex);
         getVertexUnsafe(getNumberOfVertices() - 1).applyTransform(transformToDesiredFrame, checkIfTransformInXYPlane);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         transformToDesiredFrame.transform(vertex, vertex3D);
         addVertex(vertex3D);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void update()
   {
      if (isUpToDate)
         return;

      for (int i = 0; i < numberOfVertices.getValue(); i++)
         tempVertexBuffer.get(i).set(yoVertexBuffer.get(i));
      numberOfVertices.set(EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(tempVertexBuffer, numberOfVertices.getValue()));
      for (int i = 0; i < numberOfVertices.getValue(); i++)
         yoVertexBuffer.get(i).set(tempVertexBuffer.get(i));

      isUpToDate = true;

      boundingBoxDirty = true;
      areaCentroidDirty = true;
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   private void updateCentroidAndArea()
   {
      if (areaCentroidDirty)
      {
         areaCentroidDirty = false;
         area = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(yoVertexBuffer, numberOfVertices.getValue(), clockwiseOrdered, centroid);
      }
   }

   /**
    * Updates the bounding box properties.
    */
   private void updateBoundingBox()
   {
      if (boundingBoxDirty)
      {
         boundingBoxDirty = false;
         boundingBox.setToNaN();
         boundingBox.updateToIncludePoints(this);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      YoFramePoint2D newVertex = yoVertexBuffer.get(numberOfVertices.getValue());
      if (newVertex == null)
         throw new RuntimeException("This polygon has reached its maximum number of vertices.");
      newVertex.set(x, y);
      numberOfVertices.increment();
   }

   /** {@inheritDoc} */
   @Override
   public void removeVertex(int indexOfVertexToRemove)
   {
      checkNonEmpty();
      checkIndexInBoundaries(indexOfVertexToRemove);

      if (indexOfVertexToRemove == numberOfVertices.getValue() - 1)
      {
         numberOfVertices.decrement();
         return;
      }
      isUpToDate = false;
      swap(yoVertexBuffer, indexOfVertexToRemove, numberOfVertices.getValue() - 1);
      numberOfVertices.decrement();
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends FramePoint2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices.getValue();
   }

   /**
    * Gets the maximum size that this polygon can reach.
    * <p>
    * This value is immutable.
    * </p>
    *
    * @return the maximum possible number of vertices for this polygon.
    */
   public int getMaxNumberOfVertices()
   {
      return yoVertexBuffer.size();
   }

   /**
    * Gets the {@code YoVariable} size used by this polygon.
    *
    * @return the internal reference to this polygon size variable.
    */
   public YoInteger getYoNumberOfVertices()
   {
      return numberOfVertices;
   }

   /**
    * Gets the buffer of vertices backed by {@code YoFramePoint2D}s used by this polygon.
    *
    * @return the internal reference to this polygon vertex buffer.
    */
   public List<YoFramePoint2D> getVertexBuffer()
   {
      return yoVertexBuffer;
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      checkIfUpToDate();
      updateCentroidAndArea();
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      updateCentroidAndArea();
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public FrameBoundingBox2DReadOnly getBoundingBox()
   {
      checkIfUpToDate();
      updateBoundingBox();
      return boundingBox;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void translate(double x, double y)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).add(x, y);
      }

      if (!boundingBoxDirty)
      {
         boundingBox.getMinPoint().add(x, y);
         boundingBox.getMaxPoint().add(x, y);
      }

      if (!areaCentroidDirty)
      {
         centroid.add(x, y);
      }
   }

   @Override
   public void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyTransform(transform, checkIfTransformInXYPlane);
      }

      postTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyInverseTransform(transform, checkIfTransformInXYPlane);
      }

      postTransform(transform);
   }

   private void postTransform(Transform transform)
   {
      if (numberOfVertices.getValue() <= 3)
      { // It's real cheap to update when dealing with few vertices.
         notifyVerticesChanged();
         update();
         return;
      }

      boolean updateVertices = true;

      if (transform instanceof RigidBodyTransformReadOnly)
      {
         RigidBodyTransformReadOnly rbTransform = (RigidBodyTransformReadOnly) transform;
         updateVertices = rbTransform.hasRotation();
      }
      else if (transform instanceof AffineTransformReadOnly)
      {
         AffineTransformReadOnly aTransform = (AffineTransformReadOnly) transform;
         updateVertices = aTransform.hasLinearTransform();
      }

      if (updateVertices)
      {
         for (int i = 0; i < getNumberOfVertices(); i++)
         {
            tempVertexBuffer.get(i).set(yoVertexBuffer.get(i));
         }

         // Testing ordering by looking at the convexity
         Convexity convexity = null;

         for (int vertexIndex = 0; vertexIndex < getNumberOfVertices(); vertexIndex++)
         {
            if (convexity == null)
               convexity = EuclidGeometryPolygonTools.polygon2DConvexityAtVertex(vertexIndex, tempVertexBuffer, vertexIndex, clockwiseOrdered);
            if (convexity != null)
               break;
         }

         if (convexity == Convexity.CONCAVE)
         { // The polygon got flipped, need to reverse the order to preserve the order.
            EuclidCoreTools.reverse(tempVertexBuffer, 0, getNumberOfVertices());
         }

         // Shifting vertices around to ensure the first vertex is min-x (and max-y if multiple min-xs)
         int minXMaxYVertexIndex = EuclidGeometryPolygonTools.findMinXMaxYVertexIndex(tempVertexBuffer, getNumberOfVertices());
         EuclidCoreTools.rotate(tempVertexBuffer, 0, getNumberOfVertices(), -minXMaxYVertexIndex);

         for (int i = 0; i < getNumberOfVertices(); i++)
         {
            yoVertexBuffer.get(i).set(tempVertexBuffer.get(i));
         }
      }

      // Being lazy, could transform these too.
      boundingBoxDirty = true;
      areaCentroidDirty = true;
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
   public YoFrameConvexPolygon2D duplicate(YoRegistry newRegistry)
   {
      YoInteger yoNumberOfVertices = (YoInteger) newRegistry.findVariable(numberOfVertices.getFullNameString());
      List<YoFramePoint2D> yoVertexBuffer = new ArrayList<>();
      for (int i = 0; i < this.yoVertexBuffer.size(); i++)
         yoVertexBuffer.add(this.yoVertexBuffer.get(i).duplicate(newRegistry));
      return new YoFrameConvexPolygon2D(yoVertexBuffer, yoNumberOfVertices, referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameConvexPolygon2DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameConvexPolygon2DReadOnly)
         return equals((FrameConvexPolygon2DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(Boolean.hashCode(clockwiseOrdered), vertexBufferView);
      bits = EuclidHashCodeTools.addToHashCode(bits, referenceFrame);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   private static void swap(List<YoFramePoint2D> vertexBuffer, int i, int j)
   {
      if (i == j)
         return;

      YoFramePoint2D iVertex = vertexBuffer.get(i);
      double x_i = iVertex.getX();
      double y_i = iVertex.getY();
      YoFramePoint2D jVertex = vertexBuffer.get(j);
      double x_j = jVertex.getX();
      double y_j = jVertex.getY();

      iVertex.set(x_j, y_j);
      jVertex.set(x_i, y_i);
   }
}
