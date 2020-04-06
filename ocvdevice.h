#ifndef OCVDEVICE_H
#define OCVDEVICE_H

#include "format.h"
#include "customtypes.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <numeric>

#include <opencv2/core/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking.hpp>



class OcvDevice
{
private:
    cv::VideoCapture* m_capture;
    const double mm_per_pix = static_cast<double>(REF_SIZE_MM)/static_cast<double>(REF_PIXEL);

    // internal buffers and mutexes
    cv::Mat** m_cvcap_buf;
    size_t m_cvcap_write_idx, m_cvcap_read_idx;
    std::mutex m_cvcap_mtx;

    shared_data_t* m_tracked_objects_buf = new shared_data_t();


    std::vector<double> m_vel_x, m_vel_y;


    size_t m_vel_idx = 0;

    // external refs
    std::mutex* m_mutex_ref;
    int m_cam_idx;
    cv::Mat* m_ocv_mat_buf_ref;
    size_t* m_mat_write_idx_ref;

    // threads
    std::thread m_capture_thread;
    std::thread m_segmentation_thread;
    std::thread m_tracking_thread;
    std::thread m_imshow_thread;

    tracked_object_t *m_input_objects;
    std::vector<tracked_object_t> *m_tracked_objects;
    int m_max_disappeared = 1;
    uint16_t m_id_ctr = 0;

    // cv::Mats for cv::imshow
    cv::Mat m_current_mat_cap, m_current_mat_seg, m_current_mat_tra;
    std::mutex m_mat_cap_mtx, m_mat_seg_mtx, m_mat_tra_mtx;

    // debug variables
#if (VERBOSE > 0)
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_cap;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_seg;
    std::chrono::time_point<std::chrono::_V2::steady_clock> m_start_time_tra;
#endif

    void capture_thread_func()
    {
        std::cout << "OpenCV capture thread started # " << std::this_thread::get_id() << " device: " << m_cam_idx << std::endl;
        cv::Mat image;
        m_capture = new cv::VideoCapture(m_cam_idx); // cv::CAP_V4L2
        m_capture->set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH_CV);
        m_capture->set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT_CV);
        m_capture->set(cv::CAP_PROP_FPS, FRAME_RATE_CV);
        std::cout << "Capturing with resolution " << m_capture->get(cv::CAP_PROP_FRAME_WIDTH) << " x "
                  << m_capture->get(cv::CAP_PROP_FRAME_HEIGHT) << " at " << m_capture->get(cv::CAP_PROP_FPS) << " fps" << std::endl;

        if (m_capture->isOpened())
        {
            m_segmentation_thread = std::thread(&OcvDevice::segmentation_thread_func, this);
            m_tracking_thread = std::thread(&OcvDevice::tracking_thread_func, this);
#if (IMSHOW > 0 && VERBOSE > 0)
            m_imshow_thread = std::thread(&OcvDevice::imshow_thread_func, this);
#endif
        }
#if (IMSHOW_CAP > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )  // remove imshow and waitKey later
#else
        while( m_capture->isOpened() )
#endif
        {
            // next image
            m_capture->read(image);
#if (VERBOSE > 0)
            m_start_time_cap = std::chrono::steady_clock::now();
#endif
            // calculate markers
            cv::Mat markers = cv::Mat::zeros(image.size(), CV_32S);
#if (IMSHOW_CAP > 0)
            cv::imshow("Input", image);
#endif
            prepareObjectMarkers(image, markers);
            // write to buffer
            m_cvcap_mtx.lock();
            m_cvcap_buf[m_cvcap_write_idx][0] = image;
            m_cvcap_buf[m_cvcap_write_idx++][1] = markers;
            if (m_cvcap_write_idx == BUF_SIZE_CVCAP)
                m_cvcap_write_idx = 0;
            m_cvcap_mtx.unlock();
#if (VERBOSE > 0)
            std::cout << "(OpenCV capture) Increased write index: " << m_cvcap_write_idx << " size0 " << image.size() << " type0 " << image.type() << " size1 "
                      << markers.size() << " type1 " << markers.type() << std::endl;
#endif
#if (VERBOSE > 0)
            std::cout << "(OpenCV capture) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                         (std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        }
        std::cout << "Capture thread is exiting" << std::endl;
    }

    void segmentation_thread_func()
    {
        std::cout << "OpenCV segmentation thread started # " << std::this_thread::get_id() << std::endl;
        tracked_object_t* tmp_tr_obj;
        while( m_capture->isOpened() )
        {
            if (m_cvcap_read_idx != m_cvcap_write_idx)
            {
#if (VERBOSE > 0)
                m_start_time_seg = std::chrono::steady_clock::now();
#endif
                // Get segmentation markers
                m_cvcap_mtx.lock();
                cv::Mat color = m_cvcap_buf[m_cvcap_read_idx][0];
                cv::Mat marker = m_cvcap_buf[m_cvcap_read_idx++][1];
                if (m_cvcap_read_idx == BUF_SIZE_POINTS)
                    m_cvcap_read_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased read index: " << m_cvcap_read_idx << " size0 " << color.size() << " type0 " << color.type() << " size1 "
                          << marker.size() << " type1 " << marker.type() << std::endl;
#endif
                m_cvcap_mtx.unlock();
#if (VERBOSE > 1)
                std::cout << "Before watershed: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
                // Segmenting by watershed algorithm
                marker.convertTo(marker, CV_32S);
                cv::watershed(color, marker); // 6-20 ms
                marker.convertTo(marker, CV_8U);
#if (VERBOSE > 1)
                std::cout << "After watershed: " << std::chrono::duration_cast
                             <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
                // Get object positions
                cv::Mat labels, stats, centroids;
                int marker_count = cv::connectedComponentsWithStats(marker, labels, stats, centroids, 4, CV_32S); // 10-15 ms
                int skip = 2; // !!! First two objects are just background
                int objects_count = marker_count - skip;
                if (marker_count > skip)
                {
                    tmp_tr_obj = new tracked_object_t[objects_count]();
                    // Fill array with segmented objects
                    for (int i = 0; i < objects_count; i++)
                    {
                        tmp_tr_obj[i].x = stats.at<int>(i+skip, cv::CC_STAT_LEFT);
                        tmp_tr_obj[i].y = stats.at<int>(i+skip, cv::CC_STAT_TOP);
                        tmp_tr_obj[i].w = stats.at<int>(i+skip, cv::CC_STAT_WIDTH);
                        tmp_tr_obj[i].h = stats.at<int>(i+skip, cv::CC_STAT_HEIGHT);
                        tmp_tr_obj[i].area = stats.at<int>(i+skip, cv::CC_STAT_AREA);
                        tmp_tr_obj[i].cx = centroids.at<double>(i+skip, 0);
                        tmp_tr_obj[i].cy = centroids.at<double>(i+skip, 1);
                        tmp_tr_obj[i].unique_id = -1;
                        tmp_tr_obj[i].lost_ctr = 0;
#if (IMSHOW > 0 && VERBOSE > 0)
                        // Draw markers on cv window
                        cv::rectangle(marker, cv::Rect(tmp_tr_obj[i].x, tmp_tr_obj[i].y,
                                                       tmp_tr_obj[i].w, tmp_tr_obj[i].h), cv::Scalar(0), 1);
                        cv::circle(marker, cv::Point2d(tmp_tr_obj[i].cx, tmp_tr_obj[i].cy),
                                   10, cv::Scalar(255), -1);
                        cv::putText(marker, "id: "+std::to_string(i)+" area: "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].w))
                                    + " x "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].h)),
                                    cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].cy),
                                    cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
#endif
                    }
                }
                else tmp_tr_obj = new tracked_object_t[0]();
                // Write segmented objects
                m_tracked_objects_buf->mutex.lock();
                m_tracked_objects_buf->tobj_ptr_queue.push(tmp_tr_obj);
                m_tracked_objects_buf->arr_size_queue.push(static_cast<size_t>(objects_count));
                m_tracked_objects_buf->mutex.unlock();
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased queue size: " << m_tracked_objects_buf->tobj_ptr_queue.size() << " size: array of "
                          << marker_count << std::endl;
#endif
#if (IMSHOW > 0 && VERBOSE > 0)
                cv::putText(marker, std::to_string(marker_count),
                            cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                m_mat_seg_mtx.lock();
                m_current_mat_seg = marker*(255/marker_count);
                m_mat_seg_mtx.unlock();
                m_mat_seg_mtx.lock();
                m_current_mat_cap = color;
                m_mat_seg_mtx.unlock();
#endif
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                             (std::chrono::steady_clock::now()-m_start_time_seg).count() << " ms" << std::endl;
#endif
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_SEGM));
        }
        std::cout << "Segmentation thread is exiting" << std::endl;
    }


    void tracking_thread_func()
    {
        std::cout << "OpenCV tracking thread started # " << std::this_thread::get_id() << std::endl;
        m_tracked_objects = new std::vector<tracked_object_t>();
#if (IMSHOW_TRA > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
#else
        while( m_capture->isOpened() )
#endif
        {
            if ( !m_tracked_objects_buf->tobj_ptr_queue.empty() )
            {
                // Read segmented input objects
                size_t input_objects_size, tracked_objects_size;
                m_tracked_objects_buf->mutex.lock();
                input_objects_size = m_tracked_objects_buf->arr_size_queue.front();
                m_tracked_objects_buf->arr_size_queue.pop();
                m_input_objects = new tracked_object_t[input_objects_size]();
                m_input_objects = m_tracked_objects_buf->tobj_ptr_queue.front();
                m_tracked_objects_buf->tobj_ptr_queue.pop();
                m_tracked_objects_buf->mutex.unlock();
                tracked_objects_size = m_tracked_objects->size();
#if (VERBOSE > 1)
                std::cout << "Current object centroids: " << tracked_objects_size << std::endl;
                for (size_t i = 0; i < tracked_objects_size; i++) {
                    std::cout << "idx "<< i <<" #" << m_tracked_objects->at(i).unique_id
                              << " (" << m_tracked_objects->at(i).cx << ","
                              << m_tracked_objects->at(i).cy << ")" << " lost: "
                              << m_tracked_objects->at(i).lost_ctr << std::endl;
                }
                std::cout << "Current input centroids: " << input_objects_size << std::endl;
                for (size_t i = 0; i < input_objects_size; i++) {
                    std::cout << "idx "<< i <<" #" << m_input_objects[i].unique_id << " (" << m_input_objects[i].cx
                              << "," << m_input_objects[i].cy << ")" << std::endl;
                }
#endif
                // Track objects
                std::vector<size_t> lost_objects;
                if (tracked_objects_size == 0 && input_objects_size == 0) // ||
                {
                    break;
                }
                else if (tracked_objects_size == 0 && input_objects_size > 0)
                {
                    // Register all input objects
                    for (size_t i = 0; i < input_objects_size; i++)
                    {
                        auto tmp_obj = m_input_objects[i];
                        tmp_obj.unique_id = m_id_ctr++;
                        m_tracked_objects->push_back(tmp_obj);
                    }
                }
                else if (tracked_objects_size > 0 && input_objects_size == 0)
                {
                    // Deregister all tracked objects
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        m_tracked_objects->at(i).lost_ctr++;
                        if (m_tracked_objects->at(i).lost_ctr >= m_max_disappeared)
                            lost_objects.push_back(i);
                    }
                }
                else {
                    // Find minimum distances between points
                    std::vector<std::tuple<double,int,int>> min_dist_obj_to_inp, min_dist_inp_to_obj;
                    cv::Mat distances = centroidDistances(m_tracked_objects, m_input_objects, input_objects_size);
#if (VERBOSE > 2)
                    std::cout << "Distance matrix ( " << tracked_objects_size << " x " << input_objects_size << " ):"<< std::endl << distances << std::endl;
#endif
                    double min, max;
                    int min_idx, max_idx;
                    for (int row = 0; row < static_cast<int>(tracked_objects_size); row++)
                    {
                        cv::minMaxIdx(distances.row(row).t(), &min, &max, &min_idx, &max_idx);
                        min_dist_obj_to_inp.push_back(std::make_tuple(min, row, min_idx));
                    }
                    for (int col = 0; col < static_cast<int>(input_objects_size); col++)
                    {
                        cv::minMaxIdx(distances.col(col), &min, &max, &min_idx, &max_idx);
                        min_dist_inp_to_obj.push_back(std::make_tuple(min, min_idx, col));
                    }
                    std::sort( min_dist_obj_to_inp.begin(), min_dist_obj_to_inp.end() );
                    std::sort( min_dist_inp_to_obj.begin(), min_dist_inp_to_obj.end() );

#if (VERBOSE > 2)
                    std::cout<<"Minima object -> input: "<< min_dist_obj_to_inp.size() <<std::endl;
                    for (size_t i=0;i<min_dist_obj_to_inp.size();i++) {
                        std::cout << std::get<0>(min_dist_obj_to_inp.at(i)) << " at row,col: ("
                                  << std::get<1>(min_dist_obj_to_inp.at(i)) << ","
                                  << std::get<2>(min_dist_obj_to_inp.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Minima input -> object: "<< min_dist_inp_to_obj.size() <<std::endl;
                    for (size_t i=0;i<min_dist_inp_to_obj.size();i++) {
                        std::cout << std::get<0>(min_dist_inp_to_obj.at(i)) << " at row,col: ("
                                  << std::get<1>(min_dist_inp_to_obj.at(i)) << ","
                                  << std::get<2>(min_dist_inp_to_obj.at(i)) << ")" << std::endl;
                    }
#endif
                    // Update existing points
                    std::vector<int> updated_objects;
                    std::vector<int> updated_inputs;
                    int ret;
                    for (size_t i = 0; i < min_dist_obj_to_inp.size(); i++)
                    {
                        int objidx =  std::get<1>(min_dist_obj_to_inp.at(i));
                        int inpidx = std::get<2>(min_dist_obj_to_inp.at(i));
                        ret = 0;
                        for (size_t j = 0; j < updated_inputs.size(); j++)
                        {
                            if (updated_inputs.at(j) == inpidx)
                            {
                                ret = -1;
                                // search input cols for obj idx
                                for (size_t k = 0; k < min_dist_inp_to_obj.size(); k++) {
                                    if (objidx == std::get<1>(min_dist_inp_to_obj.at(k)))
                                    {
                                        inpidx = std::get<2>(min_dist_inp_to_obj.at(k));
                                        ret = 0;
                                        break;
                                    }
                                }
                                break;
                            }
                        }

                        switch (ret) {
                        case -1:
                        {
                            m_tracked_objects->at(static_cast<size_t>(objidx)).lost_ctr++;
                            break;
                        }
                        case 0:
                        {
                            auto tmp_new_obj = m_input_objects[inpidx];
                            auto tmp_old_obj = m_tracked_objects->at(static_cast<size_t>(objidx));

                            tmp_new_obj.unique_id = tmp_old_obj.unique_id;
                            tmp_new_obj.lost_ctr = 0;
                            tmp_new_obj.vx = (tmp_new_obj.cx - tmp_old_obj.cx) / FRAME_PERIOD_CV_MS;
                            tmp_new_obj.vy = (tmp_new_obj.cy - tmp_old_obj.cy) / FRAME_PERIOD_CV_MS;
                            m_tracked_objects->at(static_cast<size_t>(objidx)) = tmp_new_obj;
                            updated_inputs.push_back(inpidx);
                            updated_objects.push_back(objidx);


                            //                            // move vel calculation somewhere else ...
                            //                            if (m_vel_x.size() < BUF_SIZE_VEL)
                            //                            {
                            //                                m_vel_x.push_back(tmp_new_obj.vx);
                            //                                m_vel_y.push_back(tmp_new_obj.vy);
                            //                                m_vel_idx++;
                            //                            }
                            //                            else
                            //                            {
                            //                                if (m_vel_idx < BUF_SIZE_VEL-1)
                            //                                {
                            //                                    m_vel_idx++;
                            //                                }
                            //                                else {
                            //                                    m_vel_idx = 0;
                            //                                }
                            //                                m_vel_x.at(m_vel_idx) = tmp_new_obj.vx;
                            //                                m_vel_y.at(m_vel_idx) = tmp_new_obj.vy;
                            //                            }
                            //                            std::cout << "Added velocity: idx " << m_vel_idx << " vx " << tmp_new_obj.vx
                            //                                      << " vy " << tmp_new_obj.vy << std::endl;



#if (VERBOSE > 1)
                            std::cout << "Updated object: idx " << objidx << " with idx " << inpidx << " #"
                                      << tmp_new_obj.unique_id << " x " << tmp_new_obj.cx
                                      << " y " << tmp_new_obj.cy << " lost "
                                      << tmp_new_obj.lost_ctr << std::endl;
#endif
                            break;
                        }
                        default: break;
                        }
                    }

                    // Register new objects
                    if (updated_inputs.size() < input_objects_size)
                    {
                        for (int i = 0; i < static_cast<int>( input_objects_size ); i++)
                        {
                            bool inp_updated = false;
                            for (size_t j = 0; j < updated_inputs.size(); j++)
                            {
                                if ( i == updated_inputs.at(j) )
                                {
                                    inp_updated = true;
                                    break;
                                }
                            }
                            if (!inp_updated)
                            {
                                auto tmp_obj = m_input_objects[i];
                                tmp_obj.unique_id = m_id_ctr++;
                                tmp_obj.lost_ctr = 0;
                                m_tracked_objects->push_back(tmp_obj);
#if (VERBOSE > 1)
                                std::cout << "New object: # " << m_tracked_objects->back().unique_id
                                          << " x " << m_tracked_objects->back().cx
                                          << " y " << m_tracked_objects->back().cy << std::endl;
#endif
                            }
                        }
                    }

                    // Deregister lost objects
                    for (size_t i=0; i<tracked_objects_size; i++)
                    {
                        auto tmp_obj = m_tracked_objects->at(i);
                        if (tmp_obj.lost_ctr >= m_max_disappeared)
                            lost_objects.push_back(i);

#if (VERBOSE >= 0)
                        // Error check if same point already exists
                        for (size_t j = 0; j < tracked_objects_size; j++)
                        {
                            if ( (i != j) && areSame(tmp_obj.cx, m_tracked_objects->at(j).cx)
                                 && areSame(tmp_obj.cy, m_tracked_objects->at(j).cy) )
                                std::cerr << "ERROR Point match: obj idx "<< j << " #"
                                          << m_tracked_objects->at(j).unique_id << " ("
                                          << m_tracked_objects->at(j).cx << ","
                                          << m_tracked_objects->at(j).cy << ") " << std::endl;
                        }
#endif
                    }
                    if ( !lost_objects.empty() )
                    {
                        deregisterObjects(&lost_objects, m_tracked_objects);
                        tracked_objects_size = m_tracked_objects->size();
                    }
#if (IMSHOW > 0 && VERBOSE > 0)

                    // Calculate mean velocity vectors
                    if ( !m_vel_x.empty() )
                    {
                        double sum_x = std::accumulate(m_vel_x.begin(), m_vel_x.end(), 0.0);
                        double sum_y = std::accumulate(m_vel_y.begin(), m_vel_y.end(), 0.0);
                        double mean_x = sum_x / m_vel_x.size();
                        double mean_y = sum_y / m_vel_y.size();
                        std::vector<double> diff_x(m_vel_x.size());
                        std::vector<double> diff_y(m_vel_y.size());
                        std::transform(m_vel_x.begin(), m_vel_x.end(), diff_x.begin(), [mean_x](double x) { return x - mean_x; });
                        std::transform(m_vel_y.begin(), m_vel_y.end(), diff_y.begin(), [mean_y](double y) { return y - mean_y; });
                        double sq_sum_x = std::inner_product(diff_x.begin(), diff_x.end(), diff_x.begin(), 0.0);
                        double sq_sum_y = std::inner_product(diff_y.begin(), diff_y.end(), diff_y.begin(), 0.0);
                        double stdev_x = std::sqrt(sq_sum_x / m_vel_x.size());
                        double stdev_y = std::sqrt(sq_sum_y / m_vel_y.size());

                        std::cout << "Velocity sum: " << sum_x << " x " << sum_y << " y" << std::endl;
                        std::cout << "Velocity mean: " << mean_x << " x " << mean_y << " y" << std::endl;
                        std::cout << "Velocity sqr sum: " << sq_sum_x << " x " << sq_sum_y << " y" << std::endl;
                        std::cout << "Velocity std dev: " << stdev_x << " x " << stdev_y << " y" << std::endl;
                    }

                    // Draw cv window
                    cv::Mat tracking_mat = cv::Mat::zeros(FRAME_HEIGHT_CV, FRAME_WIDTH_CV, CV_8U);
                    cv::putText(tracking_mat, std::to_string(tracked_objects_size), cv::Point(20, 40),
                                cv::FONT_HERSHEY_SIMPLEX, 1, 255, 3);
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        auto tmpobj = m_tracked_objects->at(i);
                        auto ctr_point = cv::Point2d(tmpobj.cx, tmpobj.cy);
                        auto vel_point = cv::Point2d(tmpobj.cx + (tmpobj.vx*FRAME_HEIGHT_CV*10),
                                                     tmpobj.cy + (tmpobj.vy*FRAME_HEIGHT_CV*10));
                        // Draw on cv::Mat
                        cv::putText(tracking_mat, "# "+std::to_string(tmpobj.unique_id),
                                    cv::Point(roundToInt(tmpobj.cx)-20, roundToInt(tmpobj.cy)-40),
                                    cv::FONT_HERSHEY_SIMPLEX, 1, 255, 3);
                        cv::rectangle(tracking_mat, cv::Rect(tmpobj.x, tmpobj.y, tmpobj.w, tmpobj.h),
                                      cv::Scalar(255), 2);
                        cv::circle(tracking_mat, ctr_point, 10, cv::Scalar(150), 2);
                        cv::arrowedLine(tracking_mat, ctr_point, vel_point, cv::Scalar(255), 2);
                    }
                    m_mat_tra_mtx.lock();
                    m_current_mat_tra = tracking_mat;
                    m_mat_tra_mtx.unlock();
#endif
#if (VERBOSE > 0)
                    std::cout << "(OpenCV tracking) Total took: " << std::chrono::duration_cast<std::chrono::milliseconds>
                                 (std::chrono::steady_clock::now()-m_start_time_tra).count() << " ms" << std::endl;
#endif
                }
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TRAC));
        }
        std::cout << "Tracking thread is exiting" << std::endl;
    }

    void imshow_thread_func()
    {
        while( m_capture->isOpened() && cv::waitKey(DELAY_SHOW) != 27 )
        {
            m_mat_cap_mtx.lock();
            cv::Mat cap = m_current_mat_cap;
            m_mat_cap_mtx.unlock();
            if ( !cap.empty() )
                cv::imshow("Capture", cap);
            m_mat_seg_mtx.lock();
            cv::Mat seg = m_current_mat_seg;
            m_mat_seg_mtx.unlock();
            if ( !seg.empty() )
                cv::imshow("Segmentation", seg);
            m_mat_tra_mtx.lock();
            cv::Mat tra = m_current_mat_tra;
            m_mat_tra_mtx.unlock();
            if ( !tra.empty() )
                cv::imshow("Tracking", tra);
            if ( cap.empty() && seg.empty() && tra.empty() )
                std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_SHOW));
        }
    }


    void deregisterObjects(std::vector<size_t> *obj_idxs, std::vector<tracked_object_t> *obj_centr)
    {
        std::sort(obj_idxs->begin(), obj_idxs->end(), std::greater<size_t>());
        for (size_t i = 0; i < obj_idxs->size(); i++) {
            auto idx = obj_idxs->at(i);
            obj_centr->erase(obj_centr->begin() + static_cast<long>(idx));
        }
    }


    cv::Mat centroidDistances(std::vector<tracked_object_t> *obj_centr, tracked_object_t *inp_centr, size_t size_in)
    {
        size_t size_obj = obj_centr->size();
        cv::Mat result = cv::Mat::zeros(static_cast<int>(size_obj), static_cast<int>(size_in), CV_64F);
        for (size_t i = 0; i < size_obj; i++)
        {
            double x_o = obj_centr->at(i).cx;
            double y_o = obj_centr->at(i).cy;
            for (size_t j = 0; j < size_in; ++j)
            {
                double dx = inp_centr[j].cx - x_o;
                double dy = inp_centr[j].cy - y_o;
                result.at<double>(static_cast<int>(i), static_cast<int>(j)) = std::sqrt(dx * dx + dy * dy);
            }
        }
        return result;
    }

public:
    OcvDevice(int idx, std::mutex* mutex, cv::Mat* buffer, size_t& write_idx_ref)
    {
        m_cam_idx = idx;
        m_mutex_ref = mutex;
        m_ocv_mat_buf_ref = buffer;
        m_mat_write_idx_ref = &write_idx_ref;

        std::cout << "New OpenCV device instance: " << m_cam_idx << " buffer: " << m_ocv_mat_buf_ref << " mutex: " << m_mutex_ref << std::endl;

        // initialize buffers
        m_cvcap_buf = new cv::Mat*[BUF_SIZE_CVCAP];
        for (int i=0; i<BUF_SIZE_CVCAP; i++)
        {
            m_cvcap_buf[i] = new cv::Mat[2];
        }
        m_cvcap_write_idx = m_cvcap_read_idx = 0;

        m_vel_x.reserve(BUF_SIZE_VEL);
        m_vel_y.reserve(BUF_SIZE_VEL);

        // Start capturing, which starts the following threads
        m_capture_thread = std::thread(&OcvDevice::capture_thread_func, this);

    }

    void prepareObjectMarkers(cv::Mat& colorimg, cv::Mat& markers)
    {
        double dist_trans_thresh = 0.1;
        cv::Mat background, dt, tmp;
        // pre processing
        cv::cvtColor(colorimg, tmp, cv::COLOR_BGR2GRAY);
        cv::threshold(tmp, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        // filtering noise with opening
        cv::morphologyEx(tmp, tmp, cv::MORPH_OPEN, cv::Mat::ones(3 , 3, CV_8U));
        // extract bg and invert
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After preprocessing: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 3); // 8-14 ms
        // cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(5,5)), cv::Point(-1,-1), 3); // 10 - 16 ms
        cv::dilate(tmp, background,  cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3)), cv::Point(-1,-1), 5); // 10-12 ms

        cv::threshold(background, background, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        // distance transformation
        cv::distanceTransform(tmp, tmp, cv::DIST_L2, 3);
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After distance transformation: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        //  cv::imshow("Normalized Distance Transformation", tmp);
        if ((false))
        { // 5-7 ms
            cv::normalize(tmp, tmp, 0, 1.0, cv::NORM_MINMAX);
            cv::threshold(tmp, tmp, dist_trans_thresh, 1, cv::THRESH_BINARY);
        }
        else {
            if ((false))
            {
                double max;
                cv::minMaxLoc(tmp, nullptr, &max);
                cv::threshold(tmp, tmp, dist_trans_thresh*max, max, cv::THRESH_BINARY);
            }
            else
            {
                cv::threshold(tmp, tmp, dist_trans_thresh*255, 255, cv::THRESH_BINARY);
            }
        }
        //   cv::imshow("Normalized Distance Transformation Thresh", tmp);
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After normalize thresh: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        // segmenting contours
        std::vector<std::vector<cv::Point>> contours;
        tmp.convertTo(tmp, CV_8U);
        findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++)
            drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
#if (VERBOSE > 1)
        std::cout << "(OpenCV capture) After find&draw contours: " << std::chrono::duration_cast
                     <std::chrono::milliseconds>(std::chrono::steady_clock::now()-m_start_time_cap).count() << " ms" << std::endl;
#endif
        // manual marker operations
        markers.convertTo(tmp, CV_8U);
        cv::bitwise_or(tmp, background, tmp);
#if (IMSHOW_CAP > 0)
        cv::imshow("Watershed markers", tmp*(255/contours.size())-10);
#endif
        tmp.convertTo(markers, CV_32S);
    }

    ~OcvDevice()
    {
        std::cout << "OcvDevice destructor" << std::endl;
        for (int i=0; i<BUF_SIZE_CVCAP; i++)
        {
            delete[] m_cvcap_buf[i];
        }
        delete[] m_cvcap_buf;

        delete[] m_tracked_objects_buf;
    }
};

#endif // OCVDEVICE_H
