#include <map_merge/FALCOExtractor.h>
#include <vector>
#include <math.h>

void scan2points(const sensor_msgs::LaserScan& scan, std::vector<Eigen::Vector2d>& points) {
    double angle_min = scan.angle_min;
    double theta = angle_min;
    for(auto r: scan.ranges){
        points.push_back(Eigen::Vector2d(r*cos(theta), r*sin(theta)));
    }
}

void getNeighPoints(sensor_msgs::LaserScan scan, std::vector<Eigen::Vector2d>& points, int candIndex, double radius, std::vector<Eigen::Vector2d>& neigh, int& midIndex)  {
    const Eigen::Vector2d& candPoint = points[candIndex];
    int alpha = std::floor(std::asin(radius / scan.ranges[candIndex]) / scan.angle_increment);
    int begIndex = std::max(0, candIndex - alpha);
    int endIndex = std::min<size_t>(candIndex + alpha + 1, scan.ranges.size());
    for (int i = begIndex; i <= endIndex; ++i) {
        if (pointsDistance(points[i], candPoint) <= radius) {
            if (i == candIndex) {
                midIndex = neigh.size();
            }
            neigh.push_back(points[i]);
        }
    }
}


FALKOExtractor::FALKOExtractor() {
    minScoreTh = 50;
    minExtractionRange = 0;
    maxExtractionRange = 30;
    subbeam = true;
    NMSRadius = 0.1;
    neighA = 0.1;
    neighB = 0.07;
    neighMinPoint = 2;
    bRatio = 2.5;
    gridSectors = 16;
}

void FALKOExtractor::extract(const sensor_msgs::LaserScan& scan, double beam_angle, std::vector<FALKO>& keypoints) {
    const int numBeams = scan.ranges.size();
    std::vector<Eigen::Vector2d> scan_points;
    std::vector<int> scores(numBeams, -10);
    std::vector<Eigen::Vector2d> neigh;
    std::vector<double> radius(numBeams);
    int neighSize;
    int neighSizeL;
    int neighSizeR;
    int midIndex;
    double triangleBLength;
    double triangleHLength;
    std::vector<double> thetaCorner(numBeams);
    int scoreL;
    int scoreR;
    int scoreMax = 0;
    std::vector<int> peaks;
    std::vector<int> neighCircularIndexesL;
    std::vector<int> neighCircularIndexesR;

    scan2points(scan, scan_points);
    for (int ind = 0; ind < numBeams; ++ind) {
        if (scan.ranges[ind] < minExtractionRange || scan.ranges[ind] > maxExtractionRange) {
            scores[ind] = -10;
            continue;
        }
        neigh.clear();
        radius[ind] = getNeighRadius(scan.ranges[ind]);
        getNeighPoints(scan, scan_points, ind, radius[ind], neigh, midIndex);
        neighSize = neigh.size();

        neighSizeL = midIndex;
        neighSizeR = neighSize - midIndex - 1;

        if (neighSizeL < neighMinPoint || neighSizeR < neighMinPoint) {
            scores[ind] = -10;
            continue;
        }

        triangleBLength = pointsDistance(neigh.front(), neigh.back());
        triangleHLength = std::abs(signedTriangleArea(neigh[midIndex], neigh.front(), neigh.back())) / triangleBLength;

        if (triangleBLength < (radius[ind] / bRatio) || triangleHLength < (radius[ind] / bRatio)) {
            scores[ind] = -10;
            continue;
        }

        thetaCorner[ind] = getCornerOrientation(neigh, midIndex);

        neighCircularIndexesL.resize(neighSizeL, 0);
        neighCircularIndexesR.resize(neighSizeR, 0);

        for (int i = 0; i < neighSizeL; ++i) {
            neighCircularIndexesL[i] = getCircularSectorIndex(neigh[i], neigh[midIndex], thetaCorner[ind]);
        }
        for (int i = 0; i < neighSizeR; ++i) {
            neighCircularIndexesR[i] = getCircularSectorIndex(neigh[midIndex + i + 1], neigh[midIndex], thetaCorner[ind]);
        }

        scoreL = 0;
        scoreR = 0;

        for (int i = midIndex - 1; i >= 0; --i) {
            for (int j = i; j >= 0; --j) {
                scoreL += circularSectorDistance(neighCircularIndexesL[i], neighCircularIndexesL[j], gridSectors);
            }
        }

        for (int i = midIndex + 1; i < neighSize; ++i) {
            for (int j = i; j < neighSize; ++j) {
                scoreR += circularSectorDistance(neighCircularIndexesR[i - midIndex - 1], neighCircularIndexesR[j - midIndex - 1], gridSectors);
            }
        }

        scores[ind] = scoreL + scoreR;

        if (scores[ind] > scoreMax) {
            scoreMax = scores[ind];
        }
    }

    for (int ind = 0; ind < numBeams; ++ind) {
        if (scores[ind] < 0) {
            scores[ind] = scoreMax;
        }
        scores[ind] = scoreMax - scores[ind];
    }

    NMSKeypoint(scores, scan, 0, numBeams, NMSRadius, (scoreMax * minScoreTh / 100.0), peaks);

    for (int i = 0; i < peaks.size(); ++i) {
        FALKO kp;
        kp.index = peaks[i];
        kp.orientation = thetaCorner[peaks[i]];
        kp.radius = radius[peaks[i]];

        Eigen::Vector2d projection;
        if (subbeam) {
            subBeamCorner(scan, scan_points, peaks[i], radius[peaks[i]], projection);

        } else {
            projection = scan_points[peaks[i]];
        }
        auto length = sqrt(projection.x()*projection.x() + projection.y()*projection.y());
        kp.point = Eigen::Vector3d(projection.x(),  projection.y(), length*tan(beam_angle));
        keypoints.push_back(std::move(kp));
    }

}

int FALKOExtractor::circularSectorDistance(int a1, int a2, int res) {
    const int r2 = res / 2;
    return std::abs(((a1 - a2) + r2) % res - r2);
}

int FALKOExtractor::getCircularSectorIndex(const Eigen::Vector2d& p, const Eigen::Vector2d& pmid, double theta) {
    return (int) floor((angleBetweenPoints(p, pmid) + M_PI - theta) / (2.0 * M_PI / gridSectors));
}

double FALKOExtractor::getNeighRadius(double rho) {
    double radius = neighA * std::exp(neighB * rho);
    if (radius >= rho) {
        return rho * 0.8;
    }
    return radius;
}

double FALKOExtractor::getCornerOrientation(const std::vector<Eigen::Vector2d>& neigh, int midIndex) {
    Eigen::Vector2d oriL(0, 0);
    Eigen::Vector2d oriR(0, 0);
    int size = neigh.size();
    for (int i = 0; i < size; ++i) {
        if (i < midIndex) {
            oriL += (neigh[i] - neigh[midIndex]); // / (s[i] - s[mid]).norm();
        } else if (i > midIndex) {
            oriR += (neigh[i] - neigh[midIndex]); // / (s[i] - s[mid]).norm();
        }
    }
    oriL /= midIndex;
    oriR /= (size - (midIndex + 1));
    Eigen::Vector2d ori = oriL + oriR;
    double theta = atan2(ori(1), ori(0));
}

void FALKOExtractor::NMSKeypoint(const std::vector<int>& scores, const sensor_msgs::LaserScan& scan, unsigned int ibeg, unsigned int iend, double radius, int minval, std::vector<int>& peaks) {
    unsigned i, imax;
    unsigned j, jbeg, jend, jmax;
    std::vector<int> candidates;
    peaks.clear();

    i = ibeg;
    imax = ibeg;
    while (i < iend) {
        int win;
        if (radius >= scan.ranges[i]) {
            win = std::floor(std::asin(0.8) / scan.angle_increment);
        } else {
            win = std::floor(std::asin(radius / scan.ranges[i]) / scan.angle_increment);
        }

        jmax = i;

        if (imax + win < i) {
            jbeg = (i >= win ? i - win : 0);
            imax = i;
        } else {
            jbeg = i;
        }

        jend = (i + win + 1 < iend ? i + win + 1 : iend);
        for (j = jbeg; j < jend; ++j) {
            if (scores[j] > scores[jmax]) {
                jmax = j;
            }
        }
        imax = (scores[jmax] >= scores[imax] ? jmax : imax);

        if (i == imax && scores[i] > minval) {
            candidates.push_back(i);
        }
        if (jmax > i) i = jmax;
        else ++i;
    }

    int i1 = 0;
    int i2 = 0;
    int counter = 0;

    for (i1 = 0, i2 = 0; i1 < candidates.size(); ++i1) {
        if (scores[candidates[i2]] == scores[candidates[i1]]) {
            counter++;
            if (2 * abs(i2 - i1) > counter) {
                ++i2;
            }
        } else {
            peaks.push_back(candidates[i2]);
            i2 = i1;
            counter = 0;
        }
    }
    if (i2 != candidates.size()) {
        peaks.push_back(candidates[i2]);
    }
}

void FALKOExtractor::subBeamCorner(const sensor_msgs::LaserScan& scan, std::vector<Eigen::Vector2d> points, int index, double radius, Eigen::Vector2d& p) {
    std::vector<Eigen::Vector2d> neigh;
    int midIndex;
    
    getNeighPoints(scan, points, index, radius, neigh, midIndex);
    int neighSize = neigh.size();

    Eigen::Vector3d leftLine;
    Eigen::Vector3d rightLine;

    std::vector<Eigen::Vector2d> leftSide;
    for (int i = 0; i <= midIndex; ++i) {
        leftSide.push_back(neigh[i]);
    }

    std::vector<Eigen::Vector2d> rightSide;
    for (int i = midIndex; i < neighSize; ++i) {
        rightSide.push_back(neigh[i]);
    }

    generateLine(leftSide, leftLine);
    generateLine(rightSide, rightLine);

    double A[4];
    double b[2];
    double x[2];
    A[0] = leftLine[0];
    A[1] = leftLine[1];
    A[2] = rightLine[0];
    A[3] = rightLine[1];

    b[0] = -leftLine[2];
    b[1] = -rightLine[2];

    bool valid = solveSystem2x2(A, b, x);

    if (not valid) {
        p = neigh[midIndex];
        return;
    }

    Eigen::Vector2d temp(x[0], x[1]);

    if (pointsDistance(neigh[midIndex], temp) < 0.20) {
        p = temp;
    } else {
        p = neigh[midIndex];
    }
}

void FALKOExtractor::generateLine(const std::vector<Eigen::Vector2d>& points, Eigen::Vector3d& model) {
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    int num = 0;
    for (unsigned int i = 0; i < points.size(); ++i) {
        sxx += points[i](0) * points[i](0);
        syy += points[i](1) * points[i](1);
        sxy += points[i](0) * points[i](1);
        sx += points[i](0);
        sy += points[i](1);
        ++num;
    }

    double msxx = (sxx - sx * sx / num) / num;
    double msyy = (syy - sy * sy / num) / num;
    double msxy = (sxy - sx * sy / num) / num;
    double b = 2.0 * msxy;
    double a = msxx;
    double c = msyy;
    double theta = 0.5 * (atan2(b, a - c) + M_PI);
    theta = atan2(sin(theta), cos(theta));
    double rho = (sx * cos(theta) + sy * sin(theta)) / num;

    if (rho < 0) {
        theta = atan2(sin(theta + M_PI), cos(theta + M_PI));
        rho = -rho;
    }
    model(0) = cos(theta);
    model(1) = sin(theta);
    model(2) = -rho;
}

bool FALKOExtractor::solveSystem2x2(double* A, double* b, double* x) {
    double det = A[0] * A[3] - A[1] * A[2];
    if (std::abs(det) < 0.0000001)
        return false;
    x[0] = (b[0] * A[3] - A[1] * b[1]) / det;
    x[1] = (A[0] * b[1] - b[0] * A[2]) / det;
    return true;
}