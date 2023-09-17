package AMM;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.*;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Zoutendijk {
    private static int feature_num;
    private int candidate_num;
    private final double lambda1;
    private final double lambda2;

    private RealMatrix E;
    private RealMatrix H;
    private RealMatrix A;

    private Zoutendijk(double lambda1, double lambda2){
        this.lambda1 = lambda1;
        this.lambda2 = lambda2;
    }

    public static RealVector getResult(double[][] Score_matrix, int candidate_num, int features, double lambda1, double lambda2) {
        feature_num = features;
        Zoutendijk compute = new Zoutendijk(lambda1, lambda2);
        compute.init(Score_matrix, candidate_num);
        return compute.start();
    }

    void init(double[][] matrix, int candidate_num) {
        this.candidate_num = candidate_num;
        //生成矩阵E
        double[][] EArrays = new double[2][feature_num + candidate_num];
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < feature_num + candidate_num; ++j) {
                EArrays[i][j] = ((i == 0 && j < feature_num) || (i == 1 && j >= feature_num)) ? 1 : 0;
            }
        }
        E = MatrixUtils.createRealMatrix(EArrays);
        //生成矩阵S
        double[][] SArrays = new double[matrix.length][matrix[0].length];
        for(int i = 0; i < feature_num; i++) {
            if (candidate_num >= 0) System.arraycopy(matrix[i], 0, SArrays[i], 0, candidate_num);
        }
        RealMatrix s = MatrixUtils.createRealMatrix(SArrays);


        RealMatrix STranspose = s.transpose();
        double[][] STArrays = STranspose.getData();

        double[][] HArrays = new double[feature_num + candidate_num][feature_num + candidate_num];
        for(int i = 0; i < feature_num + candidate_num; i++) {
            for(int j = 0; j < feature_num + candidate_num; j++) {
                if(i < feature_num && j < feature_num) {
                    HArrays[i][j] = (i == j) ? lambda1 : 0;//权重惩罚项
                }
                else if(j >=  feature_num && i < feature_num) {
                    HArrays[i][j] = -0.5 * SArrays[i][j-feature_num];
                }
                else if(j < feature_num) {
                    HArrays[i][j] = -0.5 * STArrays[i-feature_num][j];
                }
                else {
                    HArrays[i][j] = (i == j) ? lambda2 : 0;//概率惩罚项
                }
            }
        }
        H = MatrixUtils.createRealMatrix(HArrays);

        double[] AArrays = new double[feature_num + candidate_num];
        Arrays.fill(AArrays, 1.0);
        A = MatrixUtils.createRealDiagonalMatrix(AArrays);
    }

    public RealVector start() {
        //给定
        double[] xArray = new double[feature_num + candidate_num];
        xArray[feature_num-1] = 1;
        xArray[feature_num + candidate_num - 1] = 1;
        RealVector x = MatrixUtils.createRealVector(xArray);
        PointValuePair solution = null;
        //可行方向法迭代
        for(int iteration = 0; iteration < 10; iteration++) {
            List<double[]> effectiveConstrain = new ArrayList<>();
            List<double[]> nonEffectiveConstraint = new ArrayList<>();
            for(int i = 0; i < feature_num + candidate_num; i++) {
                if(Math.abs(x.dotProduct(A.getRowVector(i))) < 1e-5) {
                    effectiveConstrain.add(A.getRow(i));
                } else {
                    nonEffectiveConstraint.add(A.getRow(i));
                }
            }

            double[][] a1Arrays = new double[effectiveConstrain.size()][feature_num + candidate_num];
            double[][] a2Arrays = new double[nonEffectiveConstraint.size()][feature_num + candidate_num];

            for(int i = 0; i < effectiveConstrain.size(); i++) {
                if (feature_num + candidate_num >= 0)
                    System.arraycopy(effectiveConstrain.get(i), 0, a1Arrays[i], 0, feature_num + candidate_num);
            }
            for(int i = 0; i < nonEffectiveConstraint.size(); i++) {
                if (feature_num + candidate_num >= 0)
                    System.arraycopy(nonEffectiveConstraint.get(i), 0, a2Arrays[i], 0, feature_num + candidate_num);
            }
            List<LinearConstraint> constraints = new ArrayList<>();
            if(effectiveConstrain.size() > 0) {
                RealMatrix A1 = MatrixUtils.createRealMatrix(a1Arrays);
                for(int i = 0; i < A1.getRowDimension(); i++) {
                    constraints.add(new LinearConstraint(A1.getRow(i), Relationship.GEQ, 0.0));
                }

            }

            RealMatrix A2 = MatrixUtils.createRealMatrix(a2Arrays);
            for(int i = 0; i < 2; i++) {
                constraints.add(new LinearConstraint(E.getRowVector(i), Relationship.EQ, 0));
            }
            for(int i = 0; i < feature_num + candidate_num; i++) {
                double[] arr = new double[feature_num + candidate_num];
                Arrays.fill(arr, 0.0);
                arr[i] = 1;
                constraints.add(new LinearConstraint(arr, Relationship.GEQ, -1.0));
                constraints.add(new LinearConstraint(arr, Relationship.LEQ, 1.0));
            }


            RealVector xtH = H.scalarMultiply(2).operate(x);

            LinearObjectiveFunction f = new LinearObjectiveFunction(xtH, 0);
            try {
                solution = new SimplexSolver().optimize(f, new LinearConstraintSet(constraints), GoalType.MINIMIZE);
            }
            catch (Exception e) {
                e.printStackTrace();
            }

            if (solution != null) {
                //get solution
                double max = solution.getValue();

                if(Math.abs(max) > 1e-5) {
                    double miuMax = 0.0;
                    boolean isAllBiggerThan = true;
                    for(int i = 0; i < A2.getRowDimension(); i++) {
                        if(A2.getRowVector(i).dotProduct(MatrixUtils.createRealVector(solution.getPoint())) < 0.0) {
                            isAllBiggerThan = false;
                            break;
                        }
                    }
                    if(isAllBiggerThan) {
                        miuMax = Double.POSITIVE_INFINITY;
                    } else {
                        double[] b2 = new double[A2.getRowDimension()];
                        double[] toSub = A2.operate(x.toArray());
                        RealVector upper = MatrixUtils.createRealVector(b2).subtract(MatrixUtils.createRealVector(toSub));
                        RealVector A2d = MatrixUtils.createRealVector(A2.operate(solution.getPoint()));
                        RealVector res = upper.ebeDivide(A2d);


                        double minValue = Double.MAX_VALUE;
                        int minInd = -1;
                        for (int i = 0; i < A2d.getDimension(); i++) {
                            if (A2d.getEntry(i) < 0.0) {
                                if (res.getEntry(i) < minValue) {
                                    minValue = res.getEntry(i);
                                    minInd = i;
                                }
                            }
                        }

                        if (minInd != -1) {
                            miuMax = minValue;
                        }
                    }

//                Calculate miu0
                    double miuUpper = MatrixUtils.createRealVector(H.preMultiply(solution.getPoint())).dotProduct(x);
                    double miuLower = MatrixUtils.createRealVector(H.preMultiply(solution.getPoint())).dotProduct(MatrixUtils.createRealVector(solution.getPoint()));
                    double miu0 = - miuUpper / miuLower;

                    double[] miuList;

                    if(miu0 > miuMax || miu0 < 0) {
                        miuList = new double[]{0.0, miuMax};
                    } else {
                        miuList = new double[]{0.0, miuMax, miu0};
                    }

                    List<Double> value = new ArrayList<>();
                    for(double mu : miuList) {
                        RealVector vector = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(mu));
                        value.add(H.preMultiply(vector).dotProduct(vector));
                    }

                    int valueMinInd = -1;
                    double valueMinValue = Double.MAX_VALUE;
                    for(int i = 0; i < value.size(); i++) {
                        if(valueMinValue > value.get(i)) {
                            valueMinInd = i;
                            valueMinValue = value.get(i);
                        }
                    }

                    double miu = miuList[valueMinInd];
                    x = x.add(MatrixUtils.createRealVector(solution.getPoint()).mapMultiply(miu));
                } else {
                    break;
                }
            }
        }
        return x;

    }

}
