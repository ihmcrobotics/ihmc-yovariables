package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameUnitVector2DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * {@code FixedFrameUnitVector2DBasics} implementation which components {@code x}, {@code y} are
 * backed with {@code YoDouble}s.
 */
public class YoFrameUnitVector2D extends YoFrameTuple2D implements FixedFrameUnitVector2DBasics
{
   /** The dirty flag for this unit vector indicating whether it needs to be normalized or not. */
   private boolean dirty = false;

   /**
    * Creates a new {@code YoFrameUnitVector2D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    * <p>
    * WARNING: This constructor assumes that the variables have been previously initialized such that
    * this vector is already unitary.
    * </p>
    *
    * @param xVariable      the variable to use for the x-component.
    * @param yVariable      the variable to use for the y-component.
    * @param referenceFrame the reference frame for this vector.
    */
   public YoFrameUnitVector2D(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, referenceFrame);
      // So this vector does not modify the YoVariable by attempting to normalize them.
      dirty = false;
   }

   /**
    * Creates a new {@code YoFrameUnitVector2D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameUnitVector2D(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
      setToZero();
   }

   /**
    * Creates a new {@code YoFrameUnitVector2D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameUnitVector2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
      setToZero();
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
   public YoFrameUnitVector2D duplicate(YoRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(getYoX().getFullNameWithNameSpace());
      YoDouble y = (YoDouble) newRegistry.findVariable(getYoY().getFullNameWithNameSpace());
      return new YoFrameUnitVector2D(x, y, getReferenceFrame());
   }

   /** {@inheritDoc} */
   @Override
   public void absolute()
   {
      getYoX().set(Math.abs(getRawX()));
      getYoY().set(Math.abs(getRawY()));
   }

   /** {@inheritDoc} */
   @Override
   public void negate()
   {
      getYoX().set(-getRawX());
      getYoY().set(-getRawY());
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      if (dirty)
      {
         if (EuclidCoreTools.areAllZero(getRawX(), getRawY(), ZERO_TEST_EPSILON))
         {
            setToZero();
         }
         else
         {
            double lengthInverse = 1.0 / EuclidCoreTools.fastNorm(getRawX(), getRawY());
            getYoX().mul(lengthInverse);
            getYoY().mul(lengthInverse);
         }
         dirty = false;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void markAsDirty()
   {
      dirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDirty()
   {
      return dirty;
   }

   /** {@inheritDoc} */
   @Override
   public void set(UnitVector2DReadOnly other)
   {
      getYoX().set(other.getRawX());
      getYoY().set(other.getRawY());
      dirty = other.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      if (getRawX() != x)
      {
         getYoX().set(x);
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      if (getRawY() != y)
      {
         getYoY().set(y);
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public double getRawX()
   {
      return getYoX().getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getRawY()
   {
      return getYoY().getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      normalize();
      return super.getX();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      normalize();
      return super.getY();
   }
}
