void MainWindow::on_pb_check_connect_clicked()
{
    QString ftp_server_addr = ftp_addr + ftp_dir;

    CURL *curl;
    CURLcode res;
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (curl)
    {
        // 设置FTP服务器地址和端口
        curl_easy_setopt(curl, CURLOPT_URL, ftp_addr.toStdString().c_str());
        // 设置用户名和密码
        curl_easy_setopt(curl, CURLOPT_USERNAME, ftp_username.toStdString().c_str());
        curl_easy_setopt(curl, CURLOPT_PASSWORD, ftp_passwd.toStdString().c_str());
        // 发起连接请求
        res = curl_easy_perform(curl);

        // 检查连接状态
        if (res == CURLE_OK)
        {
            ui->Display_Edit->appendPlainText("ftp 测试连接成功 : " + QString(curl_easy_strerror(res)));
        }
        else
        {
            ui->Display_Edit->appendPlainText("ftp 测试连接失败 : " + QString(curl_easy_strerror(res)));
        }
        curl_easy_cleanup(curl);
    }
    curl_global_cleanup();
}

//用户自定义数据指针  下载总估计值 已下载   上传总值  已上传
static int progressCallback(void *p, double dltotal, double dlnow, double ult,
    double uln)
{
    Q_UNUSED(p);
    Q_UNUSED(dltotal);
    Q_UNUSED(dlnow);
    double process = (double)uln / ult * 100;
    qDebug() << "progressCallback :" << process;
    return 0;
}

//开始一个文件的上传测试  注意进度的打印
void MainWindow::on_pb_start_one_clicked()
{
    ui->Display_Edit->appendPlainText("开始上传一个文件：" + ftp_file);
    //这里实际是基于上面测试连接的基础上  加上真正的上传。
    CURL *curl;
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (curl == nullptr)
    {
        ui->Display_Edit->appendPlainText("创建句柄失败，请检查！");
        curl_global_cleanup();
        return;
    }

    FILE *hd_src = fopen(ftp_file.toStdString().c_str(), "rb");
    if (!hd_src)
    {
        ui->Display_Edit->appendPlainText("打开文件失败:" + ftp_file);
        curl_global_cleanup();
        return;
    }

    //这里是真正的上传目的 注意文件名的拼接
    QString ftp_server_addr = ftp_addr + ftp_dir + "/test";

    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_URL, ftp_server_addr.toStdString().c_str());
    curl_easy_setopt(curl, CURLOPT_USERPWD, QString(ftp_username + ":" + ftp_passwd).toStdString().c_str());

    fseek(hd_src, 0L, SEEK_END);
    long fileSize = ftell(hd_src);
    fseek(hd_src, 0L, SEEK_SET);

    curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);  //读文件的回调
    curl_easy_setopt(curl, CURLOPT_READDATA, hd_src); //设置要上传的文件的指针
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)fileSize);

    // 设置CURLOPT_NOPROGRESS为0，以启用进度回调函数
    // 设置CURLOPT_PROGRESSFUNCTION为progressCallback函数指针，用于获取上传进度
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl, CURLOPT_PROGRESSFUNCTION, progressCallback);

    CURLcode res;
    res = curl_easy_perform(curl);

    // 检查连接状态
    if (res == CURLE_OK)
    {
        ui->Display_Edit->appendPlainText("ftp 上传文件成功 : " + ftp_file);
    }
    else
    {
        ui->Display_Edit->appendPlainText("ftp 上传文件失败 : " + ftp_file);
    }
    curl_easy_cleanup(curl);
    fclose(hd_src);

    curl_global_cleanup();
}

static size_t getcontentlengthfunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    int r;
    long len = 0;

    char *pptr = (char *)ptr;
    r = sscanf(pptr, "Content-Length: %ld\n", &len);
    if (r)
        *((long *)stream) = len;

    return size * nmemb;
}

static size_t discardfunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    char *cptr = (char *)ptr;
    qDebug() << QString::fromUtf8(cptr);
    (void)ptr;
    (void)stream;
    return size * nmemb;
}

static size_t readfunc(char *ptr, size_t size, size_t nmemb, void *stream)
{
    FILE *f = static_cast<FILE *>(stream);
    size_t n;

    if (ferror(f))
        return CURL_READFUNC_ABORT;

    n = fread(ptr, size, nmemb, f) * size;

    return n;
}

//单个文件的断点续传测试 构造断点续传场景
//发现断点续传，这种方案并不可靠，分开构造curl分别获取服务端文件大小，进行续传处理
void MainWindow::on_pb_restart_one_clicked()
{
    CURL *curl = nullptr;
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (curl == nullptr)
    {
        ui->Display_Edit->appendPlainText("创建句柄失败，请检查！");
        curl_global_cleanup();
        return;
    }

    FILE *hd_src = fopen(ftp_file.toStdString().c_str(), "rb");
    if (!hd_src)
    {
        ui->Display_Edit->appendPlainText("打开文件失败:" + ftp_file);
        curl_global_cleanup();
        return;
    }
    //设置上传   url  用户名和密码 默认端口
    QString ftp_server_addr = ftp_addr + ftp_dir + "/test";
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_URL, ftp_server_addr.toStdString().c_str());
    curl_easy_setopt(curl, CURLOPT_USERPWD, QString(ftp_username + ":" + ftp_passwd).toStdString().c_str());

    long uploaded_len = 0;
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, getcontentlengthfunc);  //相应头部的回调函数
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &uploaded_len);             //从头部获取到目标远程文件的大小


    CURLcode res = CURLE_GOT_NOTHING;
    for (int i = 0; (i < 3) && (res != CURLE_OK); ++i)
    {
        curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);  //只获取响应头信息  而不实际下载响应体
        curl_easy_setopt(curl, CURLOPT_HEADER, 1L);  //响应头信息包含在返回的数据中 和上面的读数据一致

        res = curl_easy_perform(curl); //这里获取远程服务器文件的大小 
        if (res != CURLE_OK)
            continue;
        curl_easy_cleanup(curl); //获取后，先清理，再重新进行必要的设置，上传成功了。

        curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
        curl_easy_setopt(curl, CURLOPT_URL, ftp_server_addr.toStdString().c_str());
        curl_easy_setopt(curl, CURLOPT_USERPWD, QString(ftp_username + ":" + ftp_passwd).toStdString().c_str());

        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, discardfunc);            //这是获取下载的数据？
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, readfunc);
        curl_easy_setopt(curl, CURLOPT_READDATA, hd_src);             //发送请求中回调函数的指针

        curl_easy_setopt(curl, CURLOPT_FTPPORT, "-");   //默认端口
        curl_easy_setopt(curl, CURLOPT_ACCEPTTIMEOUT_MS, 7000L);
        curl_easy_setopt(curl, CURLOPT_FTP_CREATE_MISSING_DIRS, 1L);  //自动创建缺失目录
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

        ui->Display_Edit->appendPlainText("获取到服务器文件大小为：" + QString::number(uploaded_len));
        curl_easy_setopt(curl, CURLOPT_NOBODY, 0L);  //重新设置
        curl_easy_setopt(curl, CURLOPT_HEADER, 0L);

        fseek(hd_src, uploaded_len, SEEK_SET); //把hd_src从开始位置偏移uploaded_len长度
        curl_easy_setopt(curl, CURLOPT_APPEND, 1L); //远程文件存在  则追加
    }

    if (res != CURLE_OK)
    {
        curl_easy_setopt(curl, CURLOPT_APPEND, 0L);
    }

    res = curl_easy_perform(curl); //真正的数据上传

    if (res == CURLE_OK)
        ui->Display_Edit->appendPlainText("断点续传文件成功 ！");
    else
        ui->Display_Edit->appendPlainText("断点续传文件失败 ! " + QString(curl_easy_strerror(res)));

    fclose(hd_src);
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <curl/curl.h>
#include <sys/stat.h>

//
//  Make sure libcurl version > 7.32
//

// ---- common progress display ---- //
struct CustomProgress
{
    curl_off_t lastruntime; /* type depends on version, see above */
    CURL *curl;
};

// work for both download and upload
int progressCallback(void *p,
    curl_off_t dltotal,
    curl_off_t dlnow,
    curl_off_t ultotal,
    curl_off_t ulnow)
{
    struct CustomProgress *progress = (struct CustomProgress *)p;
    CURL *curl = progress->curl;
    curl_off_t curtime = 0;

    curl_easy_getinfo(curl, CURLINFO_TOTAL_TIME_T, &curtime);

    /* under certain circumstances it may be desirable for certain functionality
     to only run every N seconds, in order to do this the transaction time can
     be used */
    if ((curtime - progress->lastruntime) >= 3000000)
    {
        progress->lastruntime = curtime;
        printf("TOTAL TIME: %f \n", curtime);
    }

    // do something to display the progress
    printf("UP: %ld bytes of %ld bytes, DOWN: %ld bytes of %ld bytes \n", ulnow, ultotal, dlnow, dltotal);
    if (ultotal)
        printf("UP progress: %0.2f\n", float(ulnow / ultotal));
    if (dltotal)
        printf("DOWN progress: %0.2f\n", float(dlnow / dltotal));

    return 0;
}

// ---- upload related ---- //
// parse headers for Content-Length
size_t getContentLengthFunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    int r;
    long len = 0;

    r = sscanf((const char *)ptr, "Content-Length: %ld\n", &len);
    if (r) /* Microsoft: we don't read the specs */
        *((long *)stream) = len;
    return size * nmemb;
}

// discard already downloaded data
size_t discardFunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    return size * nmemb;
}

// read data to upload
size_t readfunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    FILE *f = (FILE *)stream;
    size_t n;
    if (ferror(f))
        return CURL_READFUNC_ABORT;
    n = fread(ptr, size, nmemb, f) * size;
    return n;
}

// do upload, will overwrite existing file
int FtpUpload(const char *remote_file_path,
    const char *local_file_path,
    const char *username,
    const char *password,
    long timeout, long tries = 3)
{
    // init curl handle
    curl_global_init(CURL_GLOBAL_ALL);
    CURL *curlhandle = curl_easy_init();

    // get user_key pair
    char user_key[1024] = { 0 };
    sprintf(user_key, "%s:%s", username, password);

    FILE *file;
    long uploaded_len = 0;
    CURLcode ret = CURLE_GOT_NOTHING;
    file = fopen(local_file_path, "rb");
    if (file == NULL)
    {
        perror(NULL);
        return 0;
    }
    curl_easy_setopt(curlhandle, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curlhandle, CURLOPT_URL, remote_file_path);
    curl_easy_setopt(curlhandle, CURLOPT_USERPWD, user_key);
    if (timeout)
        curl_easy_setopt(curlhandle, CURLOPT_FTP_RESPONSE_TIMEOUT, timeout);
    curl_easy_setopt(curlhandle, CURLOPT_HEADERFUNCTION, getContentLengthFunc);
    curl_easy_setopt(curlhandle, CURLOPT_HEADERDATA, &uploaded_len);
    curl_easy_setopt(curlhandle, CURLOPT_WRITEFUNCTION, discardFunc);
    curl_easy_setopt(curlhandle, CURLOPT_READFUNCTION, readfunc);
    curl_easy_setopt(curlhandle, CURLOPT_READDATA, file);
    curl_easy_setopt(curlhandle, CURLOPT_FTPPORT, "-"); /* disable passive mode */
    curl_easy_setopt(curlhandle, CURLOPT_FTP_CREATE_MISSING_DIRS, 1L);

    // set upload progress
    curl_easy_setopt(curlhandle, CURLOPT_XFERINFOFUNCTION, progressCallback);
    struct CustomProgress prog;
    curl_easy_setopt(curlhandle, CURLOPT_XFERINFODATA, &prog);
    curl_easy_setopt(curlhandle, CURLOPT_NOPROGRESS, 0);

    //    curl_easy_setopt(curlhandle, CURLOPT_VERBOSE, 1L); // if set 1, debug mode will print some low level msg

        // upload: 断点续传
    for (int c = 0; (ret != CURLE_OK) && (c < tries); c++)
    {
        /* are we resuming? */
        if (c)
        { /* yes */
            /* determine the length of the file already written */
            /*
            * With NOBODY and NOHEADER, libcurl will issue a SIZE
            * command, but the only way to retrieve the result is
            * to parse the returned Content-Length header. Thus,
            * getContentLengthfunc(). We need discardfunc() above
            * because HEADER will dump the headers to stdout
            * without it.
            */
            curl_easy_setopt(curlhandle, CURLOPT_NOBODY, 1L);
            curl_easy_setopt(curlhandle, CURLOPT_HEADER, 1L);
            ret = curl_easy_perform(curlhandle);
            if (ret != CURLE_OK)
                continue;
            curl_easy_setopt(curlhandle, CURLOPT_NOBODY, 0L);
            curl_easy_setopt(curlhandle, CURLOPT_HEADER, 0L);
            fseek(file, uploaded_len, SEEK_SET);
            curl_easy_setopt(curlhandle, CURLOPT_APPEND, 1L);
        }
        else
            curl_easy_setopt(curlhandle, CURLOPT_APPEND, 0L);

        ret = curl_easy_perform(curlhandle);
    }
    fclose(file);

    int curl_state = 0;
    if (ret == CURLE_OK)
        curl_state = 1;
    else
    {
        fprintf(stderr, "%s\n", curl_easy_strerror(ret));
        curl_state = 0;
    }

    // exit curl handle
    curl_easy_cleanup(curlhandle);
    curl_global_cleanup();

    return curl_state;
}

// ---- download related ---- //
// write data to upload
size_t writeFunc(void *ptr, size_t size, size_t nmemb, void *stream)
{
    std::cout << "--- write func ---" << std::endl;
    return fwrite(ptr, size, nmemb, (FILE *)stream);
}


// do download, will overwrite existing file
int FtpDownload(const char *remote_file_path,
    const char *local_file_path,
    const char *username,
    const char *password,
    long timeout = 3)
{
    // init curl handle
    curl_global_init(CURL_GLOBAL_ALL);
    CURL *curlhandle = curl_easy_init();

    // get user_key pair
    char user_key[1024] = { 0 };
    sprintf(user_key, "%s:%s", username, password);

    FILE *file;
    curl_off_t local_file_len = -1;
    long filesize = 0;
    CURLcode ret = CURLE_GOT_NOTHING;
    struct stat file_info;
    int use_resume = 0; // resume flag

    // get local file info, if success ,set resume mode
    if (stat(local_file_path, &file_info) == 0)
    {
        local_file_len = file_info.st_size;
        use_resume = 1;
    }

    // read file in append mode: 断点续传
    file = fopen(local_file_path, "ab+");
    if (file == NULL)
    {
        perror(NULL);
        return 0;
    }
    curl_easy_setopt(curlhandle, CURLOPT_URL, remote_file_path);
    curl_easy_setopt(curlhandle, CURLOPT_USERPWD, user_key); // set user:password
    // set connection timeout
    curl_easy_setopt(curlhandle, CURLOPT_CONNECTTIMEOUT, timeout);
    // set header process, get content length callback
    curl_easy_setopt(curlhandle, CURLOPT_HEADERFUNCTION, getContentLengthFunc);
    curl_easy_setopt(curlhandle, CURLOPT_HEADERDATA, &filesize);

    // 断点续传 set download resume, if use resume, set current local file length
    curl_easy_setopt(curlhandle, CURLOPT_RESUME_FROM_LARGE, use_resume ? local_file_len : 0);
    //    curl_easy_setopt(curlhandle, CURLOPT_WRITEFUNCTION, writeFunc);
    curl_easy_setopt(curlhandle, CURLOPT_WRITEDATA, file);

    // set download progress
    curl_easy_setopt(curlhandle, CURLOPT_XFERINFOFUNCTION, progressCallback);
    struct CustomProgress prog;
    curl_easy_setopt(curlhandle, CURLOPT_XFERINFODATA, &prog);
    curl_easy_setopt(curlhandle, CURLOPT_NOPROGRESS, 0);


    //    curl_easy_setopt(curlhandle, CURLOPT_VERBOSE, 1); // if set 1, debug mode will print some low level msg

    ret = curl_easy_perform(curlhandle);
    fclose(file);

    int curl_state = 0;
    if (ret == CURLE_OK)
        curl_state = 1;
    else
    {
        fprintf(stderr, "%s\n", curl_easy_strerror(ret));
        curl_state = 0;
    }

    // exit curl handle
    curl_easy_cleanup(curlhandle);
    curl_global_cleanup();

    return curl_state;
}

int GetFileList(
    const std::string &remote_file_dir,
    const std::string &local_file_dir,
    const std::string &username,
    const std::string &password,
    std::vector<std::string> &file_list,
    int timeout
)
{
    /* local file name to store the file as */
    std::string file_list_path = local_file_dir + "file_list.txt";
    FILE *ftpfile = fopen(file_list_path.c_str(), "wb");

    CURLcode res = CURLE_GOT_NOTHING;
    CURL *curl = curl_easy_init();

    if (curl)
    {
        /* get a file listing from sunet */
        curl_easy_setopt(curl, CURLOPT_URL, remote_file_dir.c_str());

        /*user & pwd*/
        curl_easy_setopt(curl, CURLOPT_USERNAME, username.c_str());
        curl_easy_setopt(curl, CURLOPT_PASSWORD, password.c_str());

        /*ftp file info will be write in this file*/
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, ftpfile);

        if (timeout)
            curl_easy_setopt(curl, CURLOPT_FTP_RESPONSE_TIMEOUT, timeout);

        /*run the opt*/
        res = curl_easy_perform(curl);

        /* check for errors */
        if (res != CURLE_OK)
        {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }

        /* always cleanup */
        curl_easy_cleanup(curl);
    }

    /* close the local file */
    fclose(ftpfile);

    /*read all lines in file*/
    std::ifstream fin(file_list_path);
    while (!fin.eof())
    {
        char strline[kMaxPathLen] = { 0 };
        fin.getline(strline, kMaxPathLen);
        file_list.push_back(strline);
    }
    fin.close();

    /*delete cache file*/
    remove(file_list_path.c_str());

    return res;
}

int main()
{
    std::cout << "==== upload test ====" << std::endl;
    FtpUpload("ftp://127.0.0.1:211/myfile/curl.zip", "/home/user/codetest/curl-7.62.0.zip", "user", "123", 1);

    std::cout << "==== download test ====" << std::endl;
    FtpDownload("ftp://127.0.0.1:211/myfile/GitHubDesktopSetup.exe", "/home/user/codetest/github.exe", "user", "123");

    return 0;
}

