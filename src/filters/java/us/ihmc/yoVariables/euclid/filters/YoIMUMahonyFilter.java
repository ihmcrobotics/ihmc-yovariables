package us.ihmc.yoVariables.euclid.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoIMUMahonyFilter implements ProcessingYoVariable
{
   private static final double MIN_MAGNITUDE = 1.0E-5;
   private static final double GRAVITY_DEFAULT_VALUE = 9.81;
   public static final Vector3DReadOnly ACCELERATION_REFERENCE;
   public static final Vector3DReadOnly NORTH_REFERENCE;
   private YoFrameVector3D rawAngularVelocity;
   private YoFrameVector3D rawLinearAcceleration;
   private YoFrameVector3D rawMagneticVector;
   private final YoFrameQuaternion estimatedOrientation;
   private final YoFrameVector3D estimatedAngularVelocity;
   private final YoFrameVector3D orientationError;
   private final YoFrameVector3D angularVelocityBias;
   private final YoDouble proportionalGain;
   private final YoDouble integralGain;
   private final DoubleProvider proportionalGainProvider;
   private final DoubleProvider integralGainProvider;
   private final YoDouble zeroAngularVelocityThreshold;
   private final YoDouble zeroLinearAccelerationThreshold;
   private final YoDouble yawRateBiasGain;
   private final double updateDT;
   private double gravityMagnitude;
   private final YoBoolean hasBeenInitialized;
   private final ReferenceFrame sensorFrame;
   private final Vector3D rotationUpdate;
   private final Quaternion quaternionUpdate;
   private final Vector3D angularVelocityUnbiased;
   private final Vector3D angularVelocityTerm;
   private boolean hasDesiredInitialHeading;
   private final Vector3D desiredInitialHeading;
   private final Vector3D m;
   private final Vector3D mRef;
   private final Vector3D a;
   private final Vector3D aRef;
   private final Vector3D normalPart;
   private final Vector3D tangentialPart;

   public YoIMUMahonyFilter(String imuName, String namePrefix, String nameSuffix, double updateDT, ReferenceFrame sensorFrame, YoRegistry parentRegistry)
   {
      this(imuName, namePrefix, nameSuffix, updateDT, false, sensorFrame, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            YoRegistry parentRegistry)
   {
      this(imuName, namePrefix, nameSuffix, updateDT, createYawRateBiasEstimator, sensorFrame, (DoubleProvider) null, (DoubleProvider) null, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            DoubleProvider proportionalGain,
                            DoubleProvider integralGain,
                            YoRegistry parentRegistry)
   {
      this(imuName,
           namePrefix,
           nameSuffix,
           updateDT,
           createYawRateBiasEstimator,
           sensorFrame,
           (YoFrameQuaternion) null,
           (YoFrameVector3D) null,
           proportionalGain,
           integralGain,
           parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            YoRegistry parentRegistry)
   {
      this(imuName, namePrefix, nameSuffix, updateDT, false, sensorFrame, estimatedOrientation, estimatedAngularVelocity, parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            YoRegistry parentRegistry)
   {
      this(imuName,
           namePrefix,
           nameSuffix,
           updateDT,
           createYawRateBiasEstimator,
           sensorFrame,
           estimatedOrientation,
           estimatedAngularVelocity,
           (DoubleProvider) null,
           (DoubleProvider) null,
           parentRegistry);
   }

   public YoIMUMahonyFilter(String imuName,
                            String namePrefix,
                            String nameSuffix,
                            double updateDT,
                            boolean createYawRateBiasEstimator,
                            ReferenceFrame sensorFrame,
                            YoFrameQuaternion estimatedOrientation,
                            YoFrameVector3D estimatedAngularVelocity,
                            DoubleProvider proportionalGain,
                            DoubleProvider integralGain,
                            YoRegistry parentRegistry)
   {
      this.gravityMagnitude = 9.81;
      this.rotationUpdate = new Vector3D();
      this.quaternionUpdate = new Quaternion();
      this.angularVelocityUnbiased = new Vector3D();
      this.angularVelocityTerm = new Vector3D();
      this.hasDesiredInitialHeading = false;
      this.desiredInitialHeading = new Vector3D();
      this.m = new Vector3D();
      this.mRef = new Vector3D();
      this.a = new Vector3D();
      this.aRef = new Vector3D();
      this.normalPart = new Vector3D();
      this.tangentialPart = new Vector3D();
      this.updateDT = updateDT;
      this.sensorFrame = sensorFrame;
      YoRegistry registry = new YoRegistry(imuName + "MahonyFilter");
      parentRegistry.addChild(registry);
      if (estimatedOrientation != null)
      {
         estimatedOrientation.checkReferenceFrameMatch(sensorFrame.getRootFrame());
      }
      else
      {
         estimatedOrientation = new YoFrameQuaternion(namePrefix, nameSuffix, sensorFrame.getRootFrame(), registry);
      }

      if (estimatedAngularVelocity != null)
      {
         estimatedAngularVelocity.checkReferenceFrameMatch(sensorFrame);
      }
      else
      {
         estimatedAngularVelocity = new YoFrameVector3D(namePrefix, nameSuffix, sensorFrame, registry);
      }

      this.estimatedOrientation = estimatedOrientation;
      this.estimatedAngularVelocity = estimatedAngularVelocity;
      this.orientationError = new YoFrameVector3D(namePrefix + "OrientationError", nameSuffix, sensorFrame, registry);
      this.angularVelocityBias = new YoFrameVector3D(namePrefix + "AngularVelocityBias", nameSuffix, sensorFrame, registry);
      if (proportionalGain == null)
      {
         this.proportionalGain = new YoDouble(namePrefix + "ProportionalGain" + nameSuffix, registry);
         this.proportionalGainProvider = this.proportionalGain;
      }
      else
      {
         this.proportionalGain = proportionalGain instanceof YoDouble ? (YoDouble) proportionalGain : null;
         this.proportionalGainProvider = proportionalGain;
      }

      if (integralGain == null)
      {
         this.integralGain = new YoDouble(namePrefix + "IntegralGain" + nameSuffix, registry);
         this.integralGainProvider = this.integralGain;
      }
      else
      {
         this.integralGain = integralGain instanceof YoDouble ? (YoDouble) integralGain : null;
         this.integralGainProvider = integralGain;
      }

      this.zeroLinearAccelerationThreshold = new YoDouble(namePrefix + "ZeroLinearAccelerationThreshold" + nameSuffix, registry);
      if (createYawRateBiasEstimator)
      {
         this.zeroAngularVelocityThreshold = new YoDouble(namePrefix + "ZeroAngularVelocityThreshold" + nameSuffix, registry);
         this.yawRateBiasGain = new YoDouble(namePrefix + "YawRateBiasGain" + nameSuffix, registry);
      }
      else
      {
         this.zeroAngularVelocityThreshold = null;
         this.yawRateBiasGain = null;
      }

      this.hasBeenInitialized = new YoBoolean(namePrefix + "HasBeenInitialized" + nameSuffix, registry);
   }

   public void setInputs(YoFrameVector3D inputAngularVelocity, YoFrameVector3D inputLinearAcceleration)
   {
      this.setInputs(inputAngularVelocity, inputLinearAcceleration, (YoFrameVector3D) null);
   }

   public void setInputs(YoFrameVector3D inputAngularVelocity, YoFrameVector3D inputLinearAcceleration, YoFrameVector3D inputMagneticVector)
   {
      if (inputAngularVelocity != null)
      {
         inputAngularVelocity.checkReferenceFrameMatch(this.sensorFrame);
      }

      if (inputLinearAcceleration != null)
      {
         inputLinearAcceleration.checkReferenceFrameMatch(this.sensorFrame);
      }

      if (inputMagneticVector != null)
      {
         inputMagneticVector.checkReferenceFrameMatch(this.sensorFrame);
      }

      this.rawAngularVelocity = inputAngularVelocity;
      this.rawLinearAcceleration = inputLinearAcceleration;
      this.rawMagneticVector = inputMagneticVector;
   }

   public void setGains(double proportionalGain, double integralGain)
   {
      this.proportionalGain.set(proportionalGain);
      this.integralGain.set(integralGain);
   }

   public void setGains(double proportionalGain, double integralGain, double zeroLinearAccelerationThreshold)
   {
      this.proportionalGain.set(proportionalGain);
      this.integralGain.set(integralGain);
      this.zeroLinearAccelerationThreshold.set(zeroLinearAccelerationThreshold);
   }

   public void setGravityMagnitude(double gravityMagnitude)
   {
      this.gravityMagnitude = gravityMagnitude;
   }

   public void setYawDriftParameters(double zeroAngularVelocityThreshold, double gain)
   {
      this.zeroAngularVelocityThreshold.set(zeroAngularVelocityThreshold);
      this.yawRateBiasGain.set(gain);
   }

   public void setHasBeenInitialized(boolean value)
   {
      this.hasBeenInitialized.set(value);
   }

   public void update()
   {
      Vector3DReadOnly inputAngularVelocity = this.rawAngularVelocity;
      Vector3DReadOnly inputLinearAcceleration = this.rawLinearAcceleration;
      Vector3DReadOnly inputMagneticVector = this.rawMagneticVector;
      if (inputMagneticVector != null)
      {
         this.update(inputAngularVelocity, inputLinearAcceleration, inputMagneticVector);
      }
      else
      {
         this.update(inputAngularVelocity, inputLinearAcceleration);
      }
   }

   public void update(FrameVector3DReadOnly inputAngularVelocity, FrameVector3DReadOnly inputLinearAcceleration)
   {
      this.update((FrameVector3DReadOnly) inputAngularVelocity, (FrameVector3DReadOnly) inputLinearAcceleration, (FrameVector3DReadOnly) null);
   }

   public void update(FrameVector3DReadOnly inputAngularVelocity, FrameVector3DReadOnly inputLinearAcceleration, FrameVector3DReadOnly inputMagneticVector)
   {
      inputAngularVelocity.checkReferenceFrameMatch(this.sensorFrame);
      inputLinearAcceleration.checkReferenceFrameMatch(this.sensorFrame);
      if (inputMagneticVector != null)
      {
         inputMagneticVector.checkReferenceFrameMatch(this.sensorFrame);
      }

      this.update((Vector3DReadOnly) inputAngularVelocity, (Vector3DReadOnly) inputLinearAcceleration, (Vector3DReadOnly) inputMagneticVector);
   }

   public void update(Vector3DReadOnly inputAngularVelocity, Vector3DReadOnly inputLinearAcceleration)
   {
      this.update((Vector3DReadOnly) inputAngularVelocity, (Vector3DReadOnly) inputLinearAcceleration, (Vector3DReadOnly) null);
   }

   public void update(Vector3DReadOnly inputAngularVelocity, Vector3DReadOnly inputLinearAcceleration, Vector3DReadOnly inputMagneticVector)
   {
      if (!this.hasBeenInitialized.getValue())
      {
         this.initialize(inputLinearAcceleration, inputMagneticVector);
      }
      else
      {
         boolean success = this.computeOrientationError(this.estimatedOrientation, inputLinearAcceleration, inputMagneticVector, this.orientationError);
         if (success)
         {
            this.angularVelocityTerm.scaleAdd(this.proportionalGainProvider.getValue(), this.orientationError, inputAngularVelocity);
            boolean hasIntegralTerm = this.updateIntegralTerm(this.angularVelocityBias,
                                                              inputMagneticVector != null,
                                                              this.estimatedOrientation,
                                                              inputAngularVelocity,
                                                              inputLinearAcceleration,
                                                              this.orientationError);
            if (hasIntegralTerm)
            {
               this.angularVelocityTerm.add(this.angularVelocityBias);
            }

            this.angularVelocityUnbiased.add(inputAngularVelocity, this.angularVelocityBias);
         }
         else
         {
            this.orientationError.setToZero();
            this.angularVelocityTerm.set(inputAngularVelocity);
            this.angularVelocityUnbiased.set(inputAngularVelocity);
         }

         this.rotationUpdate.setAndScale(this.updateDT, this.angularVelocityTerm);
         this.quaternionUpdate.setRotationVector(this.rotationUpdate);
         this.estimatedOrientation.multiply(this.quaternionUpdate);
         if (this.estimatedAngularVelocity != null)
         {
            this.estimatedAngularVelocity.set(this.angularVelocityUnbiased);
         }
      }
   }

   public void reset()
   {
      this.hasBeenInitialized.set(false);
   }

   public void setDesiredInitialHeading(Vector3DReadOnly desiredInitialHeading)
   {
      this.desiredInitialHeading.set(desiredInitialHeading);
      this.hasDesiredInitialHeading = true;
   }

   public void initialize(Orientation3DReadOnly initialOrientation)
   {
      this.estimatedOrientation.set(initialOrientation);
      this.angularVelocityBias.setToZero();
      this.hasBeenInitialized.set(true);
   }

   private void initialize(Vector3DReadOnly acceleration, Vector3DReadOnly magneticVector)
   {
      if (magneticVector == null && this.hasDesiredInitialHeading)
      {
         magneticVector = this.desiredInitialHeading;
      }

      boolean success = computeRotationMatrixFromXZAxes(magneticVector, acceleration, this.estimatedOrientation);
      if (!success)
      {
         this.estimatedOrientation.setToZero();
      }
      else
      {
         this.estimatedOrientation.invert();
      }

      this.angularVelocityBias.setToZero();
      this.hasBeenInitialized.set(true);
   }

   private boolean updateIntegralTerm(Vector3DBasics integralTerm,
                                      boolean hasMagneticVector,
                                      Orientation3DReadOnly orientation,
                                      Vector3DReadOnly angularVelocity,
                                      Vector3DReadOnly linearAcceleration,
                                      Vector3DReadOnly errorTerm)
   {
      if (Double.isFinite(this.integralGainProvider.getValue()) && !(this.integralGainProvider.getValue() <= (double) 0.0F))
      {
         orientation.inverseTransform(ACCELERATION_REFERENCE, this.aRef);
         this.a.setAndScale(this.gravityMagnitude, this.aRef);
         this.a.sub(linearAcceleration, this.a);
         if (this.zeroLinearAccelerationThreshold.getValue() > (double) 0.0F
             && this.a.lengthSquared() > MathTools.square(this.zeroLinearAccelerationThreshold.getValue()))
         {
            return true;
         }
         else
         {
            integralTerm.scaleAdd(this.integralGainProvider.getValue() * this.updateDT, errorTerm, integralTerm);
            if (hasMagneticVector)
            {
               return true;
            }
            else
            {
               if (this.yawRateBiasGain != null)
               {
                  if (angularVelocity.lengthSquared() > MathTools.square(this.zeroAngularVelocityThreshold.getValue()))
                  {
                     return true;
                  }

                  double normalPartMagnitude = TupleTools.dot(this.aRef, integralTerm);
                  if (Double.isFinite(normalPartMagnitude) && normalPartMagnitude != (double) 0.0F)
                  {
                     this.normalPart.setAndScale(normalPartMagnitude, this.aRef);
                     this.tangentialPart.sub(integralTerm, this.normalPart);
                     double yawRateError = -angularVelocity.dot(this.aRef);
                     double ajustedNormalMagnitude = EuclidCoreTools.interpolate(normalPartMagnitude, yawRateError, this.yawRateBiasGain.getValue());
                     this.normalPart.scale(ajustedNormalMagnitude / normalPartMagnitude);
                     integralTerm.add(this.normalPart, this.tangentialPart);
                  }
               }
               else
               {
                  double normalPartMagnitude = TupleTools.dot(this.aRef, integralTerm);
                  if (Double.isFinite(normalPartMagnitude) && normalPartMagnitude != (double) 0.0F)
                  {
                     this.normalPart.setAndScale(normalPartMagnitude, this.aRef);
                     this.tangentialPart.sub(integralTerm, this.normalPart);
                     this.normalPart.scale((double) 1.0F - this.integralGainProvider.getValue());
                     integralTerm.add(this.normalPart, this.tangentialPart);
                  }
               }

               return true;
            }
         }
      }
      else
      {
         integralTerm.setToZero();
         return false;
      }
   }

   private boolean computeOrientationError(QuaternionReadOnly orientation,
                                           Vector3DReadOnly acceleration,
                                           Vector3DReadOnly magneticVector,
                                           Vector3DBasics errorToPack)
   {
      boolean success = false;
      errorToPack.setToZero();
      if (magneticVector != null)
      {
         double norm = magneticVector.length();
         if (Double.isFinite(norm) && norm >= 1.0E-5)
         {
            this.m.setAndScale((double) 1.0F / norm, magneticVector);
            orientation.transform(this.m, this.mRef);
            this.mRef.setX(EuclidCoreTools.norm(this.mRef.getX(), this.mRef.getY()));
            this.mRef.setY((double) 0.0F);
            orientation.inverseTransform(this.mRef);
            errorToPack.cross(this.m, this.mRef);
            success = true;
         }
      }

      if (acceleration != null)
      {
         double norm = acceleration.length();
         if (Double.isFinite(norm) && norm >= 1.0E-5)
         {
            this.a.setAndScale((double) 1.0F / norm, acceleration);
            orientation.inverseTransform(ACCELERATION_REFERENCE, this.aRef);
            double ex = errorToPack.getX();
            double ey = errorToPack.getY();
            double ez = errorToPack.getZ();
            errorToPack.cross(this.a, this.aRef);
            errorToPack.add(ex, ey, ez);
            success = true;
         }
      }

      return success;
   }

   public YoFrameQuaternion getEstimatedOrientation()
   {
      return this.estimatedOrientation;
   }

   public YoFrameVector3D getEstimatedAngularVelocity()
   {
      return this.estimatedAngularVelocity;
   }

   public YoFrameVector3D getErrorTerm()
   {
      return this.orientationError;
   }

   public YoFrameVector3D getIntegralTerm()
   {
      return this.angularVelocityBias;
   }

   private static boolean computeRotationMatrixFromXZAxes(Vector3DReadOnly xAxis, Vector3DReadOnly zAxis, Orientation3DBasics orientationToPack)
   {
      if (xAxis == null)
      {
         xAxis = Axis3D.X;
      }

      if (zAxis == null)
      {
         zAxis = Axis3D.Z;
      }

      double zAxisX = zAxis.getX();
      double zAxisY = zAxis.getY();
      double zAxisZ = zAxis.getZ();
      double zLength = zAxis.length();
      if (zLength < 1.0E-5)
      {
         return false;
      }
      else
      {
         double invZLength = 1.0 / zLength;
         zAxisX *= invZLength;
         zAxisY *= invZLength;
         zAxisZ *= invZLength;

         // Find Y axis via cross product (Z x X)
         double yAxisX = zAxisY * xAxis.getZ() - zAxisZ * xAxis.getY();
         double yAxisY = zAxisZ * xAxis.getX() - zAxisX * xAxis.getZ();
         double yAxisZ = zAxisX * xAxis.getY() - zAxisY * xAxis.getX();

         double yLength = Math.sqrt(EuclidCoreTools.normSquared(yAxisX, yAxisY, yAxisZ));
         if (yLength < 1.0E-5)
         {
            return false;
         }
         else
         {
            // Normalize Y
            double invYLength = 1.0 / yLength;
            yAxisX *= invYLength;
            yAxisY *= invYLength;
            yAxisZ *= invYLength;

            // Find X axis via cross product (Y x Z) to ensure orthogonality
            double xAxisX = yAxisY * zAxisZ - yAxisZ * zAxisY;
            double xAxisY = yAxisZ * zAxisX - yAxisX * zAxisZ;
            double xAxisZ = yAxisX * zAxisY - yAxisY * zAxisX;
            if (orientationToPack instanceof RotationMatrixBasics)
            {
               // Fill matrix: M00, M01, M02, M10, M11, M12, M20, M21, M22
               ((RotationMatrixBasics) orientationToPack).setUnsafe(xAxisX, yAxisX, zAxisX, xAxisY, yAxisY, zAxisY, xAxisZ, yAxisZ, zAxisZ);
            }
            else
            {
               orientationToPack.setRotationMatrix(xAxisX, yAxisX, zAxisX, xAxisY, yAxisY, zAxisY, xAxisZ, yAxisZ, zAxisZ);
            }

            return true;
         }
      }
   }

   static
   {
      ACCELERATION_REFERENCE = Axis3D.Z;
      NORTH_REFERENCE = Axis3D.X;
   }
}
