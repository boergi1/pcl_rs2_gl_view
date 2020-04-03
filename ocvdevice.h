#ifndef OCVDEVICE_H
#define OCVDEVICE_H

#include "format.h"
#include "customtypes.h"

#include <iostream>
#include <thread>
#include <mutex>

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
    std::mutex m_marker_mutex;

    shared_data_t* m_tracked_objects_buf = new shared_data_t();


    size_t m_objects_write_idx, m_objects_read_idx;
    std::mutex m_objects_mutex;

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
    int m_max_disappeared = 5;
    uint16_t m_id_ctr = 0;

    // current mats for imshow
    cv::Mat m_current_mat_seg, m_current_mat_tra;
    std::mutex m_mat_seg_mtx, m_mat_tra_mtx;

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
            m_marker_mutex.lock();
            m_cvcap_buf[m_cvcap_write_idx][0] = image;
            m_cvcap_buf[m_cvcap_write_idx++][1] = markers;
            if (m_cvcap_write_idx == BUF_SIZE_CVCAP)
                m_cvcap_write_idx = 0;
#if (VERBOSE > 0)
            std::cout << "(OpenCV capture) Increased write index: " << m_cvcap_write_idx << " size0 " << image.size() << " type0 " << image.type() << " size1 "
                      << markers.size() << " type1 " << markers.type() << std::endl;
#endif
            m_marker_mutex.unlock();
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
                m_marker_mutex.lock();
                cv::Mat color = m_cvcap_buf[m_cvcap_read_idx][0];
                cv::Mat marker = m_cvcap_buf[m_cvcap_read_idx++][1];
                if (m_cvcap_read_idx == BUF_SIZE_POINTS)
                    m_cvcap_read_idx = 0;
#if (VERBOSE > 0)
                std::cout << "(OpenCV segmentation) Increased read index: " << m_cvcap_read_idx << " size0 " << color.size() << " type0 " << color.type() << " size1 "
                          << marker.size() << " type1 " << marker.type() << std::endl;
#endif
                m_marker_mutex.unlock();
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
                tmp_tr_obj = new tracked_object_t[marker_count]();

                // Fill array with segmented objects
                for (int i = 0; i < marker_count; i++)
                {
                    tmp_tr_obj[i].x = stats.at<int>(i, cv::CC_STAT_LEFT);
                    tmp_tr_obj[i].y = stats.at<int>(i, cv::CC_STAT_TOP);
                    tmp_tr_obj[i].w = stats.at<int>(i, cv::CC_STAT_WIDTH);
                    tmp_tr_obj[i].h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
                    tmp_tr_obj[i].area = stats.at<int>(i, cv::CC_STAT_AREA);
                    tmp_tr_obj[i].cx = centroids.at<double>(i, 0);
                    tmp_tr_obj[i].cy = centroids.at<double>(i, 1);
                    tmp_tr_obj[i].unique_id = -1;
                    tmp_tr_obj[i].lost_ctr = 0;
#if (IMSHOW > 0 && VERBOSE > 0)
                    // Draw markers on cv window
                    cv::rectangle(marker, cv::Rect(tmp_tr_obj[i].x, tmp_tr_obj[i].y,
                                                   tmp_tr_obj[i].w, tmp_tr_obj[i].h), cv::Scalar(0), 1);
                    cv::putText(marker, std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].w))
                                + " x "+std::to_string(static_cast<int>(mm_per_pix * tmp_tr_obj[i].h)),
                                cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y),
                                cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
                    cv::circle(marker, cv::Point(roundToInt(tmp_tr_obj[i].cx), roundToInt(tmp_tr_obj[i].cy)),
                               10, cv::Scalar(255), -1);
#endif
                }
                // Write segmented objects
                m_tracked_objects_buf->mutex.lock();
                m_tracked_objects_buf->tobj_ptr_queue.push(tmp_tr_obj);
                m_tracked_objects_buf->arr_size_queue.push(static_cast<size_t>(marker_count));
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
                for (size_t i = 0; i < tracked_objects_size; i++){
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
                std::vector<size_t> dereg_objs;
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
                            dereg_objs.push_back(i);
                    }
                }
                else {
                    // Find minimum distances between points
                    double min, max;
                    int min_idx, max_idx;
                    std::vector<std::tuple<double,int,int>> min_dist_obj_to_inp, min_dist_inp_to_obj;
                    cv::Mat distances = centroidDistances(m_tracked_objects, m_input_objects, input_objects_size);

                    for (int row = 0; row < static_cast<int>(tracked_objects_size); row++)
                    {
                        cv::minMaxIdx(distances.row(row), &min, &max, &min_idx, &max_idx);
                        min_dist_obj_to_inp.push_back(std::make_tuple(min, row, min_idx));
                    }
                    for (int col = 0; col < static_cast<int>(input_objects_size); col++)
                    {
                        cv::minMaxIdx(distances.col(col), &min, &max, &min_idx, &max_idx);
                        min_dist_inp_to_obj.push_back(std::make_tuple(min, min_idx, col));
                    }

                    std::sort( min_dist_obj_to_inp.begin(), min_dist_obj_to_inp.end() );
                    std::sort( min_dist_inp_to_obj.begin(), min_dist_inp_to_obj.end() );

                    // Update existing points
                    std::vector<int> updated_objects;
                    std::vector<int> updated_inputs;
                    int ret;

                    for (size_t i = 0; i < min_dist_obj_to_inp.size(); i++)
                    {
                        auto objidx =  std::get<1>(min_dist_obj_to_inp.at(i));
                        auto inpidx = std::get<2>(min_dist_obj_to_inp.at(i));
                        ret = 0;

                        for (size_t j = 0; j < updated_inputs.size(); j++)
                        {
                            //                           std::get<0>(tmp) == objidx || std::get<1>(tmp) == inpidx;
                            if (updated_inputs.at(j) == inpidx)
                            {
                                std::cout << "DEBUG input already used: " << inpidx << std::endl;
                                ret = 1;
                                // search input cols for obj idx
                                for (size_t k = 0; k < min_dist_inp_to_obj.size(); k++) {
                                    if (objidx == std::get<1>(min_dist_inp_to_obj.at(k)))
                                    {
                                        inpidx = std::get<2>(min_dist_inp_to_obj.at(k));
                                        std::cout << "DEBUG took alternative inp idx: " << inpidx << std::endl;
                                        ret = 2;
                                        break;
                                    }
                                }
                                break;
                            }
                        }

                        switch (ret)
                        {
                        case 0: std::cout << "DEBUG normal update" << std::endl;
                            break; case 1: std::cout << "DEBUG no partner found" << std::endl;
                            break; case 2: std::cout << "DEBUG alternative partner found" << std::endl;
                            break; default: break;
                        }

                        switch (ret)
                        {
                        case 1:
                            m_tracked_objects->at(static_cast<size_t>(objidx)).lost_ctr++;
                            break;
                        default:
                            std::cout << "DEBUG updating object " << objidx << " with input "
                                      << inpidx << std::endl;
                            auto tmp_new_obj = m_input_objects[inpidx];
                            auto tmp_old_obj = m_tracked_objects->at(static_cast<size_t>(objidx));
                            tmp_new_obj.unique_id = tmp_old_obj.unique_id;
                            tmp_new_obj.lost_ctr = 0;
                            m_tracked_objects->at(static_cast<size_t>(objidx)) = tmp_new_obj;
                            updated_inputs.push_back(inpidx);
                            updated_objects.push_back(objidx);
                            break;
                        }

                    }

                    std::cout << "DEBUG updating finished" << std::endl;
                    std::cout << "DEBUG updated objects:" << std::endl;
                    for (size_t i = 0; i < updated_objects.size(); i++) {
                        std::cout << updated_objects.at(i) << std::endl;
                    }
                    std::cout << "DEBUG updated inputs:" << std::endl;
                    for (size_t i = 0; i < updated_inputs.size(); i++) {
                        std::cout << updated_inputs.at(i) << std::endl;
                    }

                    // Register new objects
                    if (updated_inputs.size() < input_objects_size)
                    {
                        for (int i = 0; i < static_cast<int>( input_objects_size ); i++) {
                            bool inp_updated = false;
                            for (size_t j = 0; updated_inputs.size(); j++) {
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
                                std::cout << "New object: idx " << m_tracked_objects->size()-1 << " #" << m_tracked_objects->back().unique_id
                                          << " x " << m_tracked_objects->back().cx
                                          << " y " << m_tracked_objects->back().cy << std::endl;
#endif
                            }
                        }

                    }

                    // Deregister lost objects
                    std::vector<size_t> lost_objects;
                    for (size_t i=0; i<tracked_objects_size; i++) {
                        auto tmp_obj = m_tracked_objects->at(i);
                        if (tmp_obj.lost_ctr >= m_max_disappeared)
                            lost_objects.push_back(i);
#if (VERBOSE >= 0)
                        // Error check if same point already exists
                        for (size_t j = 0; j < tracked_objects_size; j++) {
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
                    // Draw cv window
                    cv::Mat tracking_mat = cv::Mat::zeros(FRAME_HEIGHT_CV, FRAME_WIDTH_CV, CV_8U);
                    for (size_t i = 0; i < tracked_objects_size; i++){
                        if (i == 0)
                        {
                            cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                                 m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(100), -1);
                            cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                               roundToInt(m_tracked_objects->at(i).cy)), 10, cv::Scalar(255), 2);
                            cv::putText(tracking_mat, std::to_string(tracked_objects_size),
                                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, 255, 3);
                        }
                        else
                        {
                            cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                               roundToInt(m_tracked_objects->at(i).cy)), 5, cv::Scalar(255), -1);
                            cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                                 m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(255), 1);
                        }
                        //                   cv::putText(tracking_mat, std::to_string((int)(mm_per_pix * tmp_tr_obj[i].w))+" x "+std::to_string((int)(mm_per_pix * tmp_tr_obj[i].h)),
                        //                               cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
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




    void tracking_thread_graveyard()
    {
        std::cout << "OpenCV GRAVE tracking thread started # " << std::this_thread::get_id() << std::endl;
        m_tracked_objects = new std::vector<tracked_object_t>();
#if (IMSHOW_TRA > 0)
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
#else
        while( m_capture->isOpened() )
#endif
        {
            if ( !m_tracked_objects_buf->tobj_ptr_queue.empty() )
            {
                size_t input_objects_size, tracked_objects_size;
                cv::Mat distances;
                std::vector<size_t> lost_objects;
                double min, max;
                cv::Point minLoc, maxLoc;

#if (VERBOSE > 0)
                m_start_time_tra = std::chrono::steady_clock::now();
#endif
                // Read segmented input objects
                m_tracked_objects_buf->mutex.lock();
                input_objects_size = m_tracked_objects_buf->arr_size_queue.front();
                m_tracked_objects_buf->arr_size_queue.pop();
                m_input_objects = new tracked_object_t[input_objects_size]();
                m_input_objects = m_tracked_objects_buf->tobj_ptr_queue.front();
                m_tracked_objects_buf->tobj_ptr_queue.pop();
                m_tracked_objects_buf->mutex.unlock();
                tracked_objects_size = m_tracked_objects->size();
#if (VERBOSE > 0)
                std::cout << "(OpenCV tracking) Reduced queue size: " << m_tracked_objects_buf->tobj_ptr_queue.size() << " size: array of " << input_objects_size << std::endl;
#endif
#if (VERBOSE > 1)
                std::cout << "Current object centroids: " << tracked_objects_size << std::endl;
                for (size_t i = 0; i < tracked_objects_size; i++){
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
                if (tracked_objects_size == 0 && input_objects_size == 0) // ||
                {
                    // Do nothing
                    std::cout << "tracking thread break" << std::endl;
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
                    std::vector<std::tuple<double,int,int>> total_min_distances,
                            row_min_distances, col_min_distances, max_distances; // value, row, col
                    // Compute centroid distances
                    distances = centroidDistances(m_tracked_objects, m_input_objects, input_objects_size);
#if (VERBOSE > 1)
                    std::cout << "Distance matrix ( " << tracked_objects_size << " x " << input_objects_size << " ):"<< std::endl << distances << std::endl;
#endif

                    // Find min/max distances of known objects (rows)
                    for (int i = 0; i < static_cast<int>(tracked_objects_size); i++)
                    {
                        cv::minMaxLoc(distances.row(i), &min, &max, &minLoc, &maxLoc);
                        row_min_distances.push_back(std::make_tuple(min, i, minLoc.x));
                        max_distances.push_back(std::make_tuple(max, i, maxLoc.x));
                    }

                    for (int i = 0; i < static_cast<int>(input_objects_size); i++)
                    {
                        cv::minMaxLoc(distances.col(i), &min, &max, &minLoc, &maxLoc);
                        col_min_distances.push_back(std::make_tuple(min, minLoc.y, i));
                    }




                    // Sort extreme values
                    total_min_distances.reserve( row_min_distances.size() + col_min_distances.size() );
                    total_min_distances.insert( total_min_distances.end(), row_min_distances.begin(), row_min_distances.end() );
                    total_min_distances.insert( total_min_distances.end(), col_min_distances.begin(), col_min_distances.end() );



                    //#ifdef remove_duplicate_vals
                    //                    std::sort( total_min_distances.begin(), total_min_distances.end() );
                    //                    total_min_distances.erase( std::unique( total_min_distances.begin(), total_min_distances.end()), total_min_distances.end() );
                    //#else
                    std::sort( total_min_distances.begin(), total_min_distances.end() );
                    // remove duplicate row/col pairs
                    total_min_distances.erase( std::unique (total_min_distances.begin(), total_min_distances.end(),
                                                            [](const std::tuple<double,int,int> &l, const std::tuple<double,int,int> &r) {
                        return (std::get<1>(l) == std::get<1>(r)) && (std::get<2>(l) == std::get<2>(r));
                    }), total_min_distances.end() );
                    //#endif

                    //#else
                    //                    // sort by cols
                    //                    std::sort( total_min_distances.begin(), total_min_distances.end(), [](const std::tuple<double,int,int> &a, const std::tuple<double,int,int> &b) {
                    //                        return (std::get<2>(a) < std::get<2>(b));
                    //                    } );
                    //                    // remove duplicate col pairs
                    //                    total_min_distances.erase( std::unique (total_min_distances.begin(), total_min_distances.end(),
                    //                                                            [](const std::tuple<double,int,int> &l, const std::tuple<double,int,int> &r) {
                    //                        return (std::get<2>(l) == std::get<2>(r));
                    //                    }), total_min_distances.end() );
                    //                    // sort by vals
                    //                    std::sort( total_min_distances.begin(), total_min_distances.end() );


                    std::sort(row_min_distances.begin(), row_min_distances.end());
                    std::sort(col_min_distances.begin(), col_min_distances.end());
                    std::sort(max_distances.begin(), max_distances.end(), std::greater<std::tuple<double,int,int>>());




#if (VERBOSE > 1)
                    std::cout<<"Minima rows: "<< row_min_distances.size() <<std::endl;
                    for (size_t i=0;i<row_min_distances.size();i++) {
                        std::cout << std::get<0>(row_min_distances.at(i)) << " at row,col: (" << std::get<1>(row_min_distances.at(i)) << ","
                                  << std::get<2>(row_min_distances.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Minima cols: "<< col_min_distances.size() <<std::endl;
                    for (size_t i=0;i<col_min_distances.size();i++) {
                        std::cout << std::get<0>(col_min_distances.at(i)) << " at row,col: (" << std::get<1>(col_min_distances.at(i)) << ","
                                  << std::get<2>(col_min_distances.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Maxima: "<< max_distances.size() << std::endl;
                    for (size_t i=0;i<max_distances.size();i++) {
                        std::cout << std::get<0>(max_distances.at(i)) << " at row,col: (" << std::get<1>(max_distances.at(i)) << ","
                                  << std::get<2>(max_distances.at(i)) << ")" << std::endl;
                    }
                    std::cout<<"Minima total: "<< total_min_distances.size() <<std::endl;
                    for (size_t i=0;i<total_min_distances.size();i++) {
                        std::cout << std::get<0>(total_min_distances.at(i)) << " at row,col: (" << std::get<1>(total_min_distances.at(i)) << ","
                                  << std::get<2>(total_min_distances.at(i)) << ")" << std::endl;
                    }
#endif










                    // Update objects
                    std::vector<size_t> used_objects;
                    std::vector<size_t> used_inputs;
                    for (size_t i = 0; i < total_min_distances.size(); i++)
                    {
                        size_t obj_idx = static_cast<size_t>(std::get<1>(total_min_distances[i]))/*row*/;
                        size_t inp_idx = static_cast<size_t>(std::get<2>(total_min_distances[i]))/*col*/;
                        auto tmp_new_obj = m_input_objects[inp_idx];
                        auto tmp_old_obj = m_tracked_objects->at(obj_idx);
                        std::cout << "DEBUG comparing object idx " << obj_idx
                                  << " with inp idx " << inp_idx << std::endl;
                        //                        if (tmp_old_obj.lost_ctr == 0)
                        //                        {
                        bool input_used = false;
                        bool object_used = false;
                        for (size_t inidx=0; inidx < used_inputs.size(); inidx++) {
                            std::cout << "DEBUG used inputs print: " << used_inputs.at(inidx) << std::endl;
                            if (inp_idx == used_inputs.at(inidx))
                            {
                                input_used = true;
                                std::cout << "DEBUG input index already used: " << inp_idx << std::endl;
                            }
                        }
                        for (size_t oidx=0; oidx < used_objects.size(); oidx++) {
                            std::cout << "DEBUG used objects print: " << used_objects.at(oidx) << std::endl;
                            if (obj_idx == used_objects.at(oidx))
                            {
                                object_used = true;
                                std::cout << "DEBUG object index already used: " << obj_idx << std::endl;
                            }
                        }

                        if (!input_used && !object_used)
                        {
                            std::cout << "DEBUG UPDATE: obj idx " << obj_idx
                                      << " with inp idx " << inp_idx << std::endl;
                            tmp_new_obj.unique_id = tmp_old_obj.unique_id;
                            m_tracked_objects->at(obj_idx) = tmp_new_obj;
                            used_inputs.push_back(inp_idx);
                            used_objects.push_back(obj_idx);
#if (VERBOSE > 1)
                            std::cout << "Updated object: idx " << obj_idx << " with idx " << inp_idx << " #"
                                      << tmp_new_obj.unique_id << " x " << tmp_new_obj.cx
                                      << " y " << tmp_new_obj.cy << " lost "
                                      << tmp_new_obj.lost_ctr << std::endl;

                            if (used_objects.size() == tracked_objects_size ||
                                    used_inputs.size() == input_objects_size)
                            {
                                std::cout << "DEBUG everthing updated" << std::endl;
                                break;
                            }
#endif
                        }
                        //                        }
                        else {
                            std::cout << "DEBUG skipping object idx " << obj_idx << std::endl;
                            if (used_objects.size() == tracked_objects_size ||
                                    used_inputs.size() == input_objects_size)
                            {
                                std::cerr << "ERROR while updating centroids" << std::endl;
                            }
                        }
                    }











                    if (tracked_objects_size > input_objects_size)
                    {

                        // Deregister specific objects















                        size_t to_delete = tracked_objects_size - input_objects_size;
                        for (size_t i = 0; i < to_delete; i++)
                        {
                            size_t obj_idx = static_cast<size_t>(std::get<1>(max_distances[i]))/*row*/;
                            if ( ++m_tracked_objects->at(obj_idx).lost_ctr >= m_max_disappeared)
                            {
                                lost_objects.push_back(obj_idx);
#if (VERBOSE > 1)
                                std::cout << "Deleting object: idx " << lost_objects.back() << " #" << m_tracked_objects->at(obj_idx).unique_id
                                          << " x " << m_tracked_objects->at(obj_idx).cx
                                          << " y " << m_tracked_objects->at(obj_idx).cy << std::endl;
#endif
                            }
#if (VERBOSE > 1)
                            else {
                                std::cout << "Lost object: idx " << obj_idx << " #"
                                          << m_tracked_objects->at(obj_idx).unique_id << " lost "
                                          << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;
                            }
#endif
                        }
                    }
                    else if (tracked_objects_size < input_objects_size)
                    {
                        // Register specific objects

















                        for (size_t i=0; i<col_min_distances.size(); i++) {
                            bool new_object = true;
                            for (size_t j=0; j<row_min_distances.size(); j++) {
                                if ( std::get<2>(col_min_distances[i]) == std::get<2>(row_min_distances[j]))
                                    new_object = false;
                            }
                            if (new_object)
                            {
                                size_t inp_idx = static_cast<size_t>(std::get<2>(col_min_distances[i]));
                                auto tmp_obj = m_input_objects[inp_idx];
                                tmp_obj.unique_id = m_id_ctr++;
                                m_tracked_objects->push_back(tmp_obj);
#if (VERBOSE > 1)
                                std::cout << "New object: idx " << m_tracked_objects->size()-1 << " #" << m_tracked_objects->back().unique_id
                                          << " x " << m_tracked_objects->back().cx
                                          << " y " << m_tracked_objects->back().cy << std::endl;
#endif
                            }

                        }
                    }










#ifdef update_by_incr_objects
                    std::vector<size_t> used_inputs;
                    for (size_t i = 0; i < tracked_objects_size; i++)
                    {
                        if (m_tracked_objects->at(i).lost_ctr == 0)
                        {
                            size_t inp_idx = std::numeric_limits<size_t>::max();
                            std::cout << "loop tracked objs with lost==0: "<< i << std::endl;
                            for (size_t j = 0; j < total_min_distances.size(); j++)
                            {
                                std::cout << "loop total mins "<< j << std::endl;
                                if (i == static_cast<size_t>(std::get<1>(total_min_distances[j])))
                                {
                                    bool used = false;
                                    std::cout << "DEBUG object " << i << " is the same as row " << j
                                              << std::endl;

                                    inp_idx = static_cast<size_t>(std::get<2>(total_min_distances[j]));

                                    for (size_t k=0;k<used_inputs.size();k++) {
                                        std::cout << "DEBUG used inputs print: " << used_inputs.at(k) << std::endl;
                                        if (inp_idx == used_inputs.at(k))
                                        {
                                            used = true;
                                            // inp_idx = static_cast<size_t>(std::get<2>(total_min_distances[i+1]));
                                            std::cout << "DEBUG input index already used: " << inp_idx << std::endl;

                                        }
                                    }
                                    if (!used) break;
                                    else std::cout << "use some other idx here" << std::endl;

                                    //  break;
                                }
                            }

                            if (inp_idx < input_objects_size)
                            {
                                std::cout << "DEBUG UPDATE: obj idx " << i << " with inp idx "
                                          << inp_idx << std::endl;
                                auto tmp_new_obj = m_input_objects[inp_idx];
                                auto tmp_old_obj = m_tracked_objects->at(i);
                                tmp_new_obj.unique_id = tmp_old_obj.unique_id;
                                tmp_new_obj.lost_ctr = tmp_old_obj.lost_ctr;
                                m_tracked_objects->at(i) = tmp_new_obj;
                                used_inputs.push_back(inp_idx);
#if (VERBOSE > 1)
                                std::cout << "Updated object: idx " << i << " with idx " << inp_idx << " #"
                                          << tmp_new_obj.unique_id << " x " << tmp_new_obj.cx
                                          << " y " << tmp_new_obj.cy << " lost "
                                          << tmp_new_obj.lost_ctr << std::endl;
#endif
                            }
                            else std::cerr << "Not a valid input index" << std::endl;
                        }

                    }
#endif


                    //                    std::vector<size_t> used_inputs;
                    //                    for (size_t i = 0; i < tracked_objects_size; i++)
                    //                    {
                    //                        size_t obj_idx = static_cast<size_t>(std::get<1>(total_min_distances[i]))/*row*/;
                    //                        size_t inp_idx = static_cast<size_t>(std::get<2>(total_min_distances[i]))/*col*/;
                    //                        auto tmp_new_obj = m_input_objects[inp_idx];
                    //                        auto tmp_old_obj = m_tracked_objects->at(obj_idx);
                    //                        if (m_tracked_objects->at(obj_idx).lost_ctr == 0)
                    //                        {
                    //                            bool input_used = false;
                    //                            for (size_t j=0;j<used_inputs.size();j++) {
                    //                                if (inp_idx == used_inputs.at(j))
                    //                                {
                    //                                    input_used = true;
                    //                                    inp_idx = static_cast<size_t>(std::get<2>(total_min_distances[i+1]));
                    //                                }
                    //                            }
                    //                            tmp_new_obj.unique_id = tmp_old_obj.unique_id;
                    //                            tmp_new_obj.lost_ctr = tmp_old_obj.lost_ctr;
                    //                            m_tracked_objects->at(obj_idx) = tmp_new_obj;
                    //                            used_inputs.push_back(inp_idx);
                    //#if (VERBOSE > 1)
                    //                            std::cout << "Updated object: idx " << obj_idx << " with idx " << inp_idx << " #"
                    //                                      << tmp_old_obj.unique_id << " x " << tmp_old_obj.cx
                    //                                      << " y " << tmp_old_obj.cy << " lost "
                    //                                      << tmp_old_obj.lost_ctr << std::endl;
                    //#endif
                    //                        }
                    //                        else
                    //                        {
                    //                            std::cout << "NOT updated object: " << obj_idx << " #"
                    //                                      << tmp_old_obj.unique_id << std::endl;
                    //                        }
                    //                    }

                    //                    // UPDATE
                    //                    std::vector<size_t> used_inputs;
                    //                    for (size_t i=0; i<row_min_distances.size(); i++) {
                    //                        //  bool skip = false;
                    //                        size_t inp_idx = static_cast<size_t>(std::get<2>(row_min_distances[i]));
                    //                        size_t obj_idx = static_cast<size_t>(std::get<1>(row_min_distances[i]));
                    //                        for (size_t j=0; j < used_inputs.size(); j++) {
                    //                            std::cout << "used inputs: " << used_inputs.at(j) << std::endl;
                    //                            if (inp_idx == used_inputs.at(j))
                    //                            {
                    //                                std::cout << "skipping: " << inp_idx << std::endl;
                    //                                //  skip = true;
                    //                                for (size_t k=0; k < col_min_distances.size(); k++) {
                    //                                    auto tmp_idx = static_cast<size_t>(std::get<2>(col_min_distances[k]));
                    //                                    if (tmp_idx != used_inputs.at(j))
                    //                                    {
                    //                                        inp_idx = tmp_idx;
                    //                                        std::cout << "using: " << inp_idx << std::endl;
                    //                                        break;
                    //                                    }
                    //                                }
                    //                            }
                    //                        }
                    //                        if (m_tracked_objects->at(obj_idx).lost_ctr == 0)
                    //                        {
                    //                            std::cout << "updating object " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                    //                                      << " with input " << inp_idx << " #" << m_input_objects[inp_idx].unique_id << std::endl;
                    //                            auto tmp_obj = m_input_objects[inp_idx];
                    //                            tmp_obj.unique_id = m_tracked_objects->at(obj_idx).unique_id;
                    //                            tmp_obj.lost_ctr = m_tracked_objects->at(obj_idx).lost_ctr;
                    //                            m_tracked_objects->at(obj_idx) = tmp_obj;
                    //                            used_inputs.push_back(inp_idx);
                    //#if (VERBOSE > 1)
                    //                            std::cout << "Updated object: idx " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                    //                                      << " x " << m_tracked_objects->at(obj_idx).cx << " y "
                    //                                      << m_tracked_objects->at(obj_idx).cy << " lost " << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;
                    //#endif
                    //                        }
                    //                        else  {
                    //                            std::cout << "NOT updating object " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                    //                                      << " with input " << inp_idx << " #" << m_input_objects[inp_idx].unique_id << std::endl;
                    //                        }
                    //                    }






                    //                // Update current objects
                    //                for (size_t i = 0; i < tracked_objects_size; i++)
                    //                {
                    //                    size_t obj_idx = static_cast<size_t>(std::get<1>(row_min_distances[i]))/*row*/;
                    //                    size_t inp_idx = static_cast<size_t>(std::get<2>(row_min_distances[i]))/*col*/;
                    //                    auto tmp_obj = m_input_objects[inp_idx];
                    //                    tmp_obj.unique_id = m_tracked_objects->at(obj_idx).unique_id;
                    //                    tmp_obj.lost_ctr = m_tracked_objects->at(obj_idx).lost_ctr;
                    //                    m_tracked_objects->at(obj_idx) = tmp_obj;
                    //#if (VERBOSE > 1)
                    //                    std::cout << "Updated object: idx " << obj_idx << " #" << m_tracked_objects->at(obj_idx).unique_id
                    //                              << " x " << m_tracked_objects->at(obj_idx).cx << " y "
                    //                              << m_tracked_objects->at(obj_idx).cy << " lost " << m_tracked_objects->at(obj_idx).lost_ctr << std::endl;
                    //#endif
                    //                }
                }

#if (VERBOSE >= 0)
                // Error check if same point already exists
                for (size_t i=0; i<tracked_objects_size; i++) {
                    auto tmp_obj = m_tracked_objects->at(i);
                    for (size_t j = 0; j < tracked_objects_size; j++) {
                        if ( (i != j) && areSame(tmp_obj.cx, m_tracked_objects->at(j).cx)
                             && areSame(tmp_obj.cy, m_tracked_objects->at(j).cy) )
                            std::cerr << "ERROR Point match: obj idx "<< j << " #"
                                      << m_tracked_objects->at(j).unique_id << " ("
                                      << m_tracked_objects->at(j).cx << ","
                                      << m_tracked_objects->at(j).cy << ") " << std::endl;
                    }
                }
#endif
                // Deregister lost objects
                if ( !lost_objects.empty() )
                {
                    deregisterObjects(&lost_objects, m_tracked_objects);
                    tracked_objects_size = m_tracked_objects->size();
                }






#if (IMSHOW > 0 && VERBOSE > 0)
                cv::Mat tracking_mat = cv::Mat::zeros(FRAME_HEIGHT_CV, FRAME_WIDTH_CV, CV_8U);
                for (size_t i = 0; i < tracked_objects_size; i++){
                    if (i == 0)
                    {
                        cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                             m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(100), -1);
                        cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                           roundToInt(m_tracked_objects->at(i).cy)), 10, cv::Scalar(255), 2);
                        cv::putText(tracking_mat, std::to_string(tracked_objects_size),
                                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, 255, 3);
                    }
                    else
                    {
                        cv::circle(tracking_mat, cv::Point(roundToInt(m_tracked_objects->at(i).cx),
                                                           roundToInt(m_tracked_objects->at(i).cy)), 5, cv::Scalar(255), -1);
                        cv::rectangle(tracking_mat, cv::Rect(m_tracked_objects->at(i).x, m_tracked_objects->at(i).y,
                                                             m_tracked_objects->at(i).w, m_tracked_objects->at(i).h), cv::Scalar(255), 1);
                    }
                    //                   cv::putText(tracking_mat, std::to_string((int)(mm_per_pix * tmp_tr_obj[i].w))+" x "+std::to_string((int)(mm_per_pix * tmp_tr_obj[i].h)),
                    //                               cv::Point(tmp_tr_obj[i].x, tmp_tr_obj[i].y), cv::FONT_HERSHEY_SIMPLEX, 1, 0, 3);
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
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TRAC));
        }
        std::cout << "Tracking thread is exiting" << std::endl;
    }

    void imshow_thread_func()
    {
        while( m_capture->isOpened() && cv::waitKey(1) != 27 )
        {
            m_mat_seg_mtx.lock();
            cv::Mat seg = m_current_mat_seg;
            m_mat_seg_mtx.unlock();
            m_mat_tra_mtx.lock();
            cv::Mat tra = m_current_mat_tra;
            m_mat_tra_mtx.unlock();
            if ( !seg.empty() && !tra.empty() )
            {
                cv::imshow("Segmentation", seg);
                cv::imshow("Tracking" , tra);
            }
            else std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_SHOW));
        }
    }


    void deregisterObjects(std::vector<size_t> *obj_idxs, std::vector<tracked_object_t> *obj_centr)
    {
        std::sort(obj_idxs->begin(), obj_idxs->end(), std::greater<size_t>());
        for (size_t i = 0; i < obj_idxs->size(); i++) {
            auto idx = obj_idxs->at(i);
            std::cout << "deleting object idx " << idx << " #" << obj_centr->at(idx).unique_id << std::endl;
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

        //   m_tracked_objects_buf = new TrackedObjectArr[BUF_SIZE_TOBJ]();
        // m_tracked_objects_buf = new tracked_object*[BUF_SIZE_TOBJ];

        //  m_tracked_objects_buf = new TrackedObjects[BUF_SIZE_TOBJ];
        m_objects_write_idx = m_objects_read_idx = 0;
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
